// Top-level controller: graph picker, fetch graph + annotations, robot controls.
import { Map2D } from './map2d.js';

const state = {
  graphName: null,
  graph: null,
  annotations: { doors: [], waypoint_pois: {}, rooms: [], no_go_zones: [], patrols: [] },
  selectedWaypointId: null,
  patrolIdx: 0,         // selected patrol in annotations.patrols
  patrolAddMode: false, // map clicks append stops to the selected patrol
  patrolOverlay: null,  // { stops, loop, current } drawn by map2d; null = hidden
  view: '2d',
  map2d: null, map3d: null,
  scans: null,        // Float32Array (Nx3) in seed frame, lazy-loaded
  scansVisible: false,
  loadToken: 0,       // increments per loadGraph; late responses bail if mismatched
  live: {
    available: false,   // server has --live session
    enabled: false,     // user toggled it on
    pose: null,         // last poll result
    mode: 'select',     // 'select' | 'drive' | 'go': drive sends nav commands on waypoint
                        // click, go on a click anywhere on the map
    maxGoalDistance: null,  // meters from a waypoint the server accepts in go mode
    uploadedGraph: null,
    pollTimer: null,
    look: null,         // { waypointId, headingDeg, stepDeg } for the 2D overlay; null = hidden
    lease: null,        // last /api/lease result: { ours, holder, stale }; null = unknown
  },
};

function setStatus(msg) {
  const el = document.getElementById('status');
  el.textContent = msg;
  if (msg) setTimeout(() => { if (el.textContent === msg) el.textContent = ''; }, 3000);
}
function api(path, opts) { return fetch(`/api/graphs/${encodeURIComponent(state.graphName)}${path}`, opts); }

async function init() {
  // Check if the server was started with --live; if so, expose the toggle.
  try {
    const live = await fetch('/api/live').then(r => r.json());
    state.live.available = !!live.enabled;
    state.live.uploadedGraph = live.uploaded_graph;
    state.live.maxGoalDistance = live.max_goal_distance_m;
    if (state.live.available) {
      document.getElementById('live-toggle').hidden = false;
      document.getElementById('tab-drive').hidden = false;
      document.getElementById('tab-move').hidden = false;
      document.getElementById('estop-block').hidden = false;
      document.getElementById('tab-drive').click();
      // Independent of the Live toggle, keep the estop and lease state fresh so
      // the operator always knows if the robot is armed/cut and who controls it.
      pollControl();
      setInterval(pollControl, 1000);
    }
  } catch { /* offline-only; ignore */ }

  const graphs = await fetch('/api/graphs').then(r => r.json());
  const sel = document.getElementById('graph-select');
  graphs.forEach(g => {
    const opt = document.createElement('option');
    opt.value = g.name;
    opt.textContent = g.name;
    sel.appendChild(opt);
  });
  sel.addEventListener('change', () => loadGraph(sel.value));
  sel.value = graphs[0].name;
  await loadGraph(graphs[0].name);
}

async function loadGraph(name) {
  const token = ++state.loadToken;
  state.graphName = name;
  state.selectedWaypointId = null;
  state.map2d?.destroy?.();
  state.map3d?.destroy?.();
  state.map2d = null;
  state.map3d = null;
  state.scans = null; state.scansVisible = false;
  document.getElementById('scans-toggle').textContent = 'Show scans';

  const graph = await api('').then(r => r.json());
  if (state.loadToken !== token) return;
  const annotations = await api('/annotations').then(r => r.json());
  if (state.loadToken !== token) return;

  state.graph = graph;
  state.annotations = annotations;
  document.getElementById('graph-info').textContent =
    `${state.graph.waypoints.length} waypoints · ${state.graph.fiducials.length} fiducials · anchored=${state.graph.anchored}`;

  // Reset 2D
  const c2 = document.getElementById('canvas2d');
  state.map2d = new Map2D(c2, state.graph, state, {
    onSelectWaypoint: wp => selectWaypoint(wp),
    onSelectFiducial: f => selectFiducial(f),
    goMode: () => state.live.enabled && state.live.mode === 'go' && !state.patrolAddMode,
    onGoToPoint: (x, y, headingDeg) => goToPoint(x, y, headingDeg),
  });
  state.map2d.draw();

  // Reset 3D container
  const c3 = document.getElementById('canvas3d');
  c3.innerHTML = '';

  // If currently in 3D, rebuild it now
  if (state.view === '3d') await ensure3D(token);
  if (state.loadToken !== token) return;

  state.patrolIdx = 0;
  setPatrolAddMode(false);
  renderPatrols();
}

// --- Tabs ---
document.querySelectorAll('#tabs .tab').forEach(tab => {
  tab.addEventListener('click', () => {
    document.querySelectorAll('#tabs .tab').forEach(t => t.classList.remove('active'));
    document.querySelectorAll('.panel').forEach(p => p.classList.remove('active'));
    tab.classList.add('active');
    document.getElementById(`panel-${tab.dataset.tab}`).classList.add('active');
    syncCamera();
    // Adding stops by map click only makes sense while the patrol is on screen.
    if (tab.dataset.tab !== 'patrol') setPatrolAddMode(false);
    refreshPatrolOverlay();
    state.map2d?.draw();
  });
});

// --- Go to a point ---
async function goToPoint(x, y, headingDeg) {
  if (state.live.uploadedGraph !== state.graphName) {
    setStatus('Upload graph to robot first (Drive tab).');
    return;
  }
  const where = `(${x.toFixed(2)}, ${y.toFixed(2)})`;
  const facing = headingDeg == null ? '' : `, facing ${Math.round((headingDeg + 360) % 360)}°`;
  if (!confirm(`Send Spot to ${where}${facing}?`)) return;
  const q = new URLSearchParams({ x: x.toFixed(3), y: y.toFixed(3) });
  if (headingDeg != null) q.set('heading_deg', headingDeg.toFixed(2));
  try {
    const r = await api(`/navigate_point?${q}`, { method: 'POST' });
    const j = await r.json().catch(() => ({}));
    setStatus(r.ok ? `Going to ${where}` : `Go failed: ${j.detail || r.status}`);
  } catch (e) {
    setStatus('Network error while sending the goal.');
  }
}

// --- Map clicks ---
function selectWaypoint(wp) {
  if (state.patrolAddMode && currentPatrol()) {
    addPatrolStop(wp.id);
    setStatus(`Added stop ${currentPatrol().stops.length}: ${waypointLabel(wp.id)} (unsaved)`);
    return;
  }
  if (state.live.enabled && state.live.mode === 'drive') {
    if (state.live.uploadedGraph !== state.graphName) {
      setStatus('Upload graph to robot first (Drive tab).');
      return;
    }
    if (!confirm(`Navigate Spot to waypoint ${wp.name || wp.id}?`)) return;
    api(`/navigate/${encodeURIComponent(wp.id)}`, { method: 'POST' })
      .then(async r => setStatus(r.ok ? `Navigating to ${wp.id}` : `Nav failed: ${(await r.json()).detail || r.status}`));
    return;
  }
  state.selectedWaypointId = wp.id;
  setStatus(`${waypointLabel(wp.id)} (${wp.id})`);
  state.map2d?.draw();
}

function selectFiducial(f) {
  setStatus(`Tag #${f.tag_id}: ${f.category}`);
}

async function ensure3D(token) {
  const c3 = document.getElementById('canvas3d');
  if (!state.map3d) {
    const { Map3D } = await import('./map3d.js');
    if (token !== undefined && state.loadToken !== token) return;
    state.map3d = new Map3D(c3, state.graph, state, {
      onSelectWaypoint: wp => selectWaypoint(wp),
      onSelectFiducial: f => selectFiducial(f),
      apiBase: () => `/api/graphs/${encodeURIComponent(state.graphName)}`,
    });
    if (state.scansVisible && state.scans) state.map3d.setScans(state.scans);
  } else {
    state.map3d.refit();
  }
}

document.getElementById('scans-toggle').addEventListener('click', async () => {
  const btn = document.getElementById('scans-toggle');
  if (!state.scans) {
    const token = state.loadToken;
    btn.disabled = true;
    btn.textContent = 'Loading scans…';
    setStatus('Fetching point clouds (this may take a moment)…');
    const r = await api('/all_points');
    const buf = await r.arrayBuffer();
    if (state.loadToken !== token) { btn.disabled = false; return; }
    state.scans = new Float32Array(buf);
    btn.disabled = false;
    setStatus(`Loaded ${(state.scans.length/3).toLocaleString()} points`);
  }
  state.scansVisible = !state.scansVisible;
  btn.textContent = state.scansVisible ? 'Hide scans' : 'Show scans';
  state.map2d?.setScans(state.scansVisible ? state.scans : null);
  state.map3d?.setScans(state.scansVisible ? state.scans : null);
  state.map2d?.draw();
});

document.getElementById('view-toggle').addEventListener('click', async () => {
  const c2 = document.getElementById('canvas2d'), c3 = document.getElementById('canvas3d');
  if (state.view === '2d') {
    c2.style.display = 'none'; c3.style.display = 'block';
    state.view = '3d';
    document.getElementById('view-toggle').textContent = 'Switch to 2D';
    await ensure3D();
  } else {
    c3.style.display = 'none'; c2.style.display = 'block';
    state.view = '2d';
    document.getElementById('view-toggle').textContent = 'Switch to 3D';
    state.map2d?.draw();
  }
});

// --- Live mode ---

function navStatusText(ns) {
  let t = ns.state;
  if (ns.goal) t += ` (${ns.goal.x.toFixed(1)}, ${ns.goal.y.toFixed(1)})`;
  if (ns.heading_deg != null) t += ` → ${Math.round(ns.heading_deg)}°`;
  if (ns.sweep) t += ` (sweep ${ns.sweep.index + 1}/${ns.sweep.total}, ${ns.sweep.phase})`;
  if (ns.patrol) {
    const p = ns.patrol;
    t += ` (patrol ${p.name} ${p.index + 1}/${p.total}${p.loop ? `, lap ${p.lap}` : ''}, ${p.phase})`;
  }
  if (ns.error) t += ` (${ns.error})`;
  return t;
}

// One big indicator for the operator. Every nav state collapses into
// in progress / arrived / unable to complete, plus idle before any command.
const NAV_BANNER_KIND = {
  navigating: 'progress', sweeping: 'progress', patrolling: 'progress',
  reached: 'arrived', idle: 'idle',
};   // anything else (stuck, lost, no_route, canceled, error, ...) → 'unable'
const NAV_BANNER_LABEL = {
  progress: 'In progress', arrived: 'Arrived', unable: 'Unable to complete', idle: 'Idle',
};
const NAV_FAIL_REASON = {
  stuck: 'Spot is stuck', lost: 'Spot got lost', no_route: 'No route to the goal',
  no_localization: 'Spot is not localized', overridden: 'Overridden by another command',
  canceled: 'Stopped by operator', error: 'Robot error',
};

function updateNavBanner() {
  const el = document.getElementById('nav-banner');
  const ns = state.live.enabled ? state.live.navStatus : null;
  const kind = ns ? (NAV_BANNER_KIND[ns.state] || 'unable') : 'idle';
  el.className = kind;
  document.getElementById('nav-banner-label').textContent = NAV_BANNER_LABEL[kind];

  let detail;
  if (!ns) {
    detail = 'Turn Live on to see navigation status.';
  } else if (kind === 'idle') {
    detail = 'No navigation command yet.';
  } else {
    const wp = ns.destination ? waypointLabel(ns.destination) : null;
    const face = ns.heading_deg != null ? `, facing ${Math.round(ns.heading_deg)}°` : '';
    const p = ns.patrol, sw = ns.sweep;
    const place = ns.goal ? `(${ns.goal.x.toFixed(1)}, ${ns.goal.y.toFixed(1)})` : wp;
    let what = place ? `${place}${face}` : '';
    if (p) what = `Patrol "${p.name}" stop ${p.index + 1}/${p.total}${p.loop ? ` (lap ${p.lap})` : ''}: ${what}`;
    else if (sw) what = `Look-around ${sw.index + 1}/${sw.total} at ${what}`;
    if (kind === 'progress') {
      const holding = (p || sw)?.phase === 'dwelling';
      detail = p || sw ? `${what}${holding ? ', holding' : ''}` : `Going to ${what}`;
    } else if (kind === 'arrived') {
      detail = p ? `Patrol "${p.name}" finished at ${wp}` : sw ? `Look-around done at ${wp}` : `At ${what}`;
    } else {
      const reason = NAV_FAIL_REASON[ns.state] || ns.state;
      detail = `${reason}${what ? `: ${what}` : ''}${ns.error ? ` (${ns.error})` : ''}`;
    }
  }
  document.getElementById('nav-banner-detail').textContent = detail;
}

function updateDriveStatusUI() {
  const el = document.getElementById('drive-status');
  if (!state.live.available) {
    el.textContent = 'Server was started without --live. No robot connection.';
    return;
  }
  const matches = state.live.uploadedGraph === state.graphName;
  const parts = [
    `Live: ${state.live.enabled ? 'on' : 'off'}`,
    `Uploaded graph: ${state.live.uploadedGraph || '(none)'}`,
    matches ? 'Loaded graph matches uploaded.' : 'Loaded graph does NOT match uploaded — upload it before navigating.',
    state.live.pose?.localized ? `Localized at ${state.live.pose.waypoint_id.slice(0, 8)}…` : 'Not localized.',
    state.live.navStatus ? `Nav: ${navStatusText(state.live.navStatus)}` : '',
  ];
  el.innerHTML = parts.join('<br>');
}

async function pollPose() {
  try {
    const r = await fetch('/api/pose');
    if (!r.ok) throw new Error(r.statusText);
    state.live.pose = await r.json();
  } catch {
    state.live.pose = null;
  }
  try {
    state.live.navStatus = await fetch('/api/nav_status').then(r => r.json());
  } catch { /* ignore */ }
  try {
    state.live.estop = await fetch('/api/estop/state').then(r => r.json());
  } catch { state.live.estop = null; }
  updateDriveStatusUI();
  updateNavBanner();
  updateEstopUI();
  updateLookUI();
  refreshPatrolOverlay();
  updatePatrolUI();
  state.map2d?.draw();
  state.map3d?.setLivePose?.(state.live.pose);
  // Update header badge
  const badge = document.getElementById('live-badge');
  if (!state.live.enabled) { badge.hidden = true; return; }
  badge.hidden = false;
  if (!state.live.pose) {
    badge.textContent = '● disconnected';
    badge.style.color = '#fc8181';
  } else if (!state.live.pose.localized) {
    badge.textContent = '● not localized';
    badge.style.color = '#a0aec0';
  } else {
    badge.textContent = `● live (${state.live.pose.x.toFixed(1)}, ${state.live.pose.y.toFixed(1)})`;
    badge.style.color = '#48bb78';
  }
}

document.getElementById('live-toggle').addEventListener('click', () => {
  state.live.enabled = !state.live.enabled;
  document.getElementById('live-toggle').textContent = `Live: ${state.live.enabled ? 'on' : 'off'}`;
  if (state.live.enabled) {
    if (state.live.pollTimer) clearInterval(state.live.pollTimer);
    state.live.pollTimer = setInterval(pollPose, 200);
    pollPose();
  } else {
    if (state.live.pollTimer) clearInterval(state.live.pollTimer);
    state.live.pollTimer = null;
    state.live.pose = null;
    document.getElementById('live-badge').hidden = true;
    updateNavBanner();
    updateLookUI();
    refreshPatrolOverlay();
    state.map2d?.draw();
    state.map3d?.setLivePose?.(null);
    document.getElementById('camera-on').checked = false;
    syncCamera();
  }
});

function setDriveMode(mode) {
  state.live.mode = mode;
  document.getElementById('drive-mode').checked = mode === 'drive';
  document.getElementById('drive-go-mode').checked = mode === 'go';
  // Click-to-navigate requires Live polling to be on (it's what gates the
  // nav request and drives pose/estop status) — checking either box is
  // useless on its own if the user forgot to flip the header toggle, so
  // turn it on for them instead of silently no-op'ing on map clicks.
  if (mode !== 'select' && !state.live.enabled) {
    document.getElementById('live-toggle').click();
  }
  setStatus({
    drive: 'Drive mode: click a waypoint to send Spot there.',
    go: `Go mode: click within ${state.live.maxGoalDistance} m of a waypoint to send Spot there.`,
    select: 'Select mode.',
  }[mode]);
  state.map2d?.draw();
}
document.getElementById('drive-mode').addEventListener('change', e =>
  setDriveMode(e.target.checked ? 'drive' : 'select'));
document.getElementById('drive-go-mode').addEventListener('change', e =>
  setDriveMode(e.target.checked ? 'go' : 'select'));

async function powerOn() {
  setStatus('Powering on motors…');
  const r = await fetch('/api/power_on', { method: 'POST' });
  const j = await r.json().catch(() => ({}));
  setStatus(r.ok ? `Powered on: ${j.powered_on}` : `Power-on failed: ${j.detail || r.status}`);
}

async function stand() {
  setStatus('Standing…');
  try {
    const r = await fetch('/api/stand', { method: 'POST' });
    const j = await r.json().catch(() => ({}));
    setStatus(r.ok ? 'Standing' : `Stand failed: ${j.detail || r.status}`);
  } catch (e) {
    setStatus('Network error while trying to stand.');
  }
}

async function sit() {
  setStatus('Sitting…');
  try {
    const r = await fetch('/api/sit', { method: 'POST' });
    const j = await r.json().catch(() => ({}));
    setStatus(r.ok ? 'Sitting' : `Sit failed: ${j.detail || r.status}`);
  } catch (e) {
    setStatus('Network error while trying to sit.');
  }
}

async function stopRobot() {
  const r = await fetch('/api/stop', { method: 'POST' });
  const j = await r.json().catch(() => ({}));
  setStatus(r.ok ? 'STOP sent' : `STOP failed: ${j.detail || r.status}`);
}

for (const tab of ['drive', 'move']) {
  document.getElementById(`${tab}-power-on`).addEventListener('click', powerOn);
  document.getElementById(`${tab}-stand`).addEventListener('click', stand);
  document.getElementById(`${tab}-sit`).addEventListener('click', sit);
  document.getElementById(`${tab}-stop`).addEventListener('click', stopRobot);
}

document.getElementById('drive-upload').addEventListener('click', async () => {
  if (!state.graphName) return;
  setStatus('Uploading graph to robot…');
  const r = await api('/upload', { method: 'POST' });
  const j = await r.json().catch(() => ({}));
  if (!r.ok) { setStatus(`Upload failed: ${j.detail || r.status}`); return; }
  state.live.uploadedGraph = state.graphName;
  setStatus(`Uploaded ${j.waypoints} waypoints, ${j.edges} edges (${j.waypoint_snapshots_uploaded} snapshots streamed)`);
  updateDriveStatusUI();
});

document.getElementById('drive-localize').addEventListener('click', async () => {
  if (!state.graphName) return;
  setStatus('Localizing via nearest fiducial…');
  const r = await api('/localize', { method: 'POST' });
  const j = await r.json().catch(() => ({}));
  setStatus(r.ok ? `Localized at ${j.waypoint_id?.slice(0, 8)}…` : `Localize failed: ${j.detail || r.status}`);
});

// --- Move ---
// One press is one short step: the server commands a fixed velocity that ends by
// itself. The pad stays locked until that step is over, so holding Enter or
// clicking fast can't stack commands into continuous motion.
const movePad = () => document.querySelectorAll('#move-pad [data-move]');

async function moveStep(direction) {
  const el = document.getElementById('move-status');
  movePad().forEach(b => { b.disabled = true; });
  try {
    const r = await fetch(`/api/move/${direction}`, { method: 'POST' });
    const j = await r.json().catch(() => ({}));
    if (r.ok) {
      el.textContent = `Moved: ${direction.replace('_', ' ')}`;
      await new Promise(done => setTimeout(done, j.duration_s * 1000));
    } else {
      el.textContent = `Move failed: ${j.detail || r.status}`;
    }
  } catch (e) {
    el.textContent = 'Network error while moving.';
  }
  movePad().forEach(b => { b.disabled = false; });
}
movePad().forEach(b => b.addEventListener('click', () => moveStep(b.dataset.move)));

// --- Look around ---
// Turn in place to a map-frame heading at the waypoint the robot just reached
// (or, before any nav, the waypoint it's localized to). Headings use the same
// convention as pose.yaw: 0° = map +x, counter-clockwise positive. The dial is
// drawn in the 2D map's orientation, so where a button sits on the dial is
// where Spot will face on the map. The server turns heading → GraphNav goal.
const HEADING_STEP_DEG = 30;
const LOOK_ACTIVE_STATES = ['navigating', 'sweeping', 'patrolling', 'reached'];
const MOVING_STATES = ['navigating', 'sweeping', 'patrolling'];
const TILT_STEP_DEG = 10, MAX_TILT_DEG = 30;   // server rejects beyond ±30
const DIAL_C = 110, DIAL_R = 84;   // SVG viewBox is 220×220

function buildLookDial() {
  const c = DIAL_C, r = DIAL_R;
  let html = `<circle class="ring" cx="${c}" cy="${c}" r="${r}"/>
    <g id="look-needle" visibility="hidden">
      <line x1="${c}" y1="${c}" x2="${c + r - 30}" y2="${c}"/>
      <polygon points="${c + r - 20},${c} ${c + r - 32},${c - 6} ${c + r - 32},${c + 6}"/>
    </g>
    <circle class="hub" cx="${c}" cy="${c}" r="4"/>`;
  for (let h = 0; h < 360; h += HEADING_STEP_DEG) {
    const a = h * Math.PI / 180;
    const x = c + r * Math.cos(a), y = c - r * Math.sin(a);   // SVG y is down
    html += `<g class="heading" data-heading="${h}" transform="translate(${x.toFixed(1)} ${y.toFixed(1)})">
      <title>Face ${h}°</title><circle r="15"/><text>${h}°</text></g>`;
  }
  const svg = document.getElementById('look-dial');
  svg.innerHTML = html;
  svg.querySelectorAll('.heading').forEach(g =>
    g.addEventListener('click', () => lookAt(Number(g.dataset.heading))));
}

function lookTargetId() {
  const ns = state.live.navStatus;
  if (ns?.destination && LOOK_ACTIVE_STATES.includes(ns.state)) return ns.destination;
  return state.live.pose?.localized ? state.live.pose.waypoint_id : null;
}

function waypointLabel(id) {
  const wp = state.graph?.waypoints.find(w => w.id === id);
  return state.annotations.waypoint_pois[id]?.name || wp?.name || id.slice(0, 8);
}

function updateLookUI() {
  const pose = state.live.pose, ns = state.live.navStatus;
  const id = state.live.enabled ? lookTargetId() : null;
  const ready = !!id && state.live.uploadedGraph === state.graphName
    && !!state.graph?.waypoints.some(w => w.id === id);
  const active = ready && ns?.destination === id && LOOK_ACTIVE_STATES.includes(ns.state);
  const heading = active ? ns.heading_deg : null;
  const settled = active && (ns.state === 'reached' || ns.sweep?.phase === 'dwelling'
    || ns.patrol?.phase === 'dwelling');
  state.live.look = ready ? { waypointId: id, headingDeg: heading, stepDeg: HEADING_STEP_DEG } : null;

  const svg = document.getElementById('look-dial');
  svg.classList.toggle('disabled', !ready);
  document.getElementById('look-sweep').disabled = !ready;
  svg.querySelectorAll('.heading').forEach(g => {
    const on = heading != null && Number(g.dataset.heading) === Math.round(heading) % 360;
    g.classList.toggle('active', on && !settled);
    g.classList.toggle('reached', on && settled);
  });

  const needle = document.getElementById('look-needle');
  const headingEl = document.getElementById('look-heading');
  const hint = '0° → right, 90° ↑ up, as on the 2D map.';
  if (state.live.enabled && pose?.localized) {
    const deg = pose.yaw * 180 / Math.PI;
    needle.setAttribute('visibility', 'visible');
    needle.setAttribute('transform', `rotate(${(-deg).toFixed(1)} ${DIAL_C} ${DIAL_C})`);
    headingEl.textContent = `Robot heading ${Math.round((deg + 360) % 360)}° (cyan). ${hint}`;
  } else {
    needle.setAttribute('visibility', 'hidden');
    headingEl.textContent = `Map frame. ${hint}`;
  }

  // Tilt needs only a live, standing robot, not localization. Disabled while
  // moving because GraphNav re-levels the body on every command it re-issues.
  const tilt = ns?.tilt_deg ?? 0;
  const canTilt = state.live.enabled && !MOVING_STATES.includes(ns?.state);
  document.getElementById('look-tilt-up').disabled = !canTilt || tilt >= MAX_TILT_DEG;
  document.getElementById('look-tilt-level').disabled = !canTilt || tilt === 0;
  document.getElementById('look-tilt-down').disabled = !canTilt || tilt <= -MAX_TILT_DEG;
  document.getElementById('look-tilt').textContent =
    tilt > 0 ? `${tilt}° up` : tilt < 0 ? `${-tilt}° down` : 'level';

  const targetEl = document.getElementById('look-target');
  if (!state.live.enabled) { targetEl.textContent = 'Turn Live on to use.'; return; }
  if (!id) { targetEl.textContent = 'Localize the robot first.'; return; }
  if (!ready) { targetEl.textContent = 'Upload this graph to the robot first.'; return; }
  let detail;
  if (!active) {
    detail = 'Localized here, not navigated. A turn first steps onto this waypoint.';
  } else if (ns.patrol) {
    const p = ns.patrol;
    detail = `Patrol "${p.name}" stop ${p.index + 1}/${p.total}: ${p.phase}`
      + (heading != null ? `, facing ${Math.round(heading)}°` : '');
  } else if (ns.sweep) {
    const step = `Sweep ${ns.sweep.index + 1}/${ns.sweep.total}`;
    detail = ns.state === 'reached' ? `${step} done.`
      : `${step}: ${ns.sweep.phase === 'dwelling' ? `holding ${ns.sweep.dwell_s}s at` : 'turning to'} ${Math.round(heading)}°`;
  } else if (ns.state === 'reached') {
    detail = heading != null ? `Arrived, facing ${Math.round(heading)}°.` : 'Arrived. Pick a heading to face.';
  } else {
    detail = heading != null ? `Going to face ${Math.round(heading)}°…` : 'En route. Pick a heading to face on arrival.';
  }
  targetEl.textContent = `Waypoint ${waypointLabel(id)}\n${detail}`;
}

async function lookAt(heading) {
  const id = lookTargetId();
  if (!id) return;
  setStatus(`Turning to ${heading}° at ${waypointLabel(id)}…`);
  const r = await api(`/navigate/${encodeURIComponent(id)}?heading_deg=${heading}`, { method: 'POST' });
  if (!r.ok) {
    const j = await r.json().catch(() => ({}));
    setStatus(`Turn failed: ${j.detail || r.status}`);
  }
}

document.getElementById('look-sweep').addEventListener('click', async () => {
  const id = lookTargetId();
  if (!id) return;
  const dwell = Math.max(0, parseFloat(document.getElementById('look-dwell').value) || 0);
  const n = 360 / HEADING_STEP_DEG;
  if (!confirm(`Turn Spot through all ${n} headings at ${waypointLabel(id)}, holding ${dwell}s at each?`)) return;
  const r = await api(`/sweep/${encodeURIComponent(id)}?step_deg=${HEADING_STEP_DEG}&dwell_s=${dwell}`,
    { method: 'POST' });
  const j = await r.json().catch(() => ({}));
  setStatus(r.ok ? `Sweeping ${n} headings at ${waypointLabel(id)}` : `Sweep failed: ${j.detail || r.status}`);
});

async function tiltTo(deg) {
  deg = Math.max(-MAX_TILT_DEG, Math.min(MAX_TILT_DEG, deg));
  const r = await fetch(`/api/tilt?pitch_deg=${deg}`, { method: 'POST' });
  const j = await r.json().catch(() => ({}));
  if (!r.ok) { setStatus(`Tilt failed: ${j.detail || r.status}`); return; }
  // Don't wait for the next poll, so quick repeated clicks keep stepping.
  if (state.live.navStatus) state.live.navStatus.tilt_deg = deg;
  updateLookUI();
  setStatus(deg ? `Looking ${deg > 0 ? 'up' : 'down'} ${Math.abs(deg)}°` : 'Body level');
}
const currentTilt = () => state.live.navStatus?.tilt_deg ?? 0;
document.getElementById('look-tilt-up').addEventListener('click', () => tiltTo(currentTilt() + TILT_STEP_DEG));
document.getElementById('look-tilt-level').addEventListener('click', () => tiltTo(0));
document.getElementById('look-tilt-down').addEventListener('click', () => tiltTo(currentTilt() - TILT_STEP_DEG));

buildLookDial();
updateLookUI();

// --- Patrol ---
// A patrol is an ordered list of stops {waypoint_id, heading_deg, dwell_s},
// saved in annotations.json with the map's other annotations. Running one
// sends the in-memory copy (saved or not); the server drives to each stop,
// faces its heading (null = the waypoint's recorded orientation) and holds
// for its dwell, on the same worker as navigate / look-around.
const DEFAULT_DWELL_S = 5;

function currentPatrol() {
  return state.annotations.patrols[state.patrolIdx] || null;
}

function setPatrolAddMode(on) {
  state.patrolAddMode = on;
  document.getElementById('patrol-add-mode').checked = on;
}

function renderPatrols() {
  const patrols = state.annotations.patrols;
  state.patrolIdx = Math.min(Math.max(state.patrolIdx, 0), patrols.length - 1);
  const sel = document.getElementById('patrol-select');
  sel.innerHTML = '';
  patrols.forEach((p, i) => sel.add(new Option(p.name || `(unnamed ${i + 1})`, i)));
  sel.value = String(state.patrolIdx);
  sel.hidden = !patrols.length;
  const p = currentPatrol();
  document.getElementById('patrol-empty').hidden = !!p;
  document.getElementById('patrol-editor').hidden = !p;
  if (!p) {
    setPatrolAddMode(false);
    refreshPatrolOverlay();
    return;
  }
  document.getElementById('patrol-name').value = p.name;
  document.getElementById('patrol-loop').checked = !!p.loop;
  renderPatrolStops();
}

function headingSelect(selected) {
  const sel = document.createElement('select');
  sel.add(new Option('as recorded', ''));
  for (let h = 0; h < 360; h += HEADING_STEP_DEG) sel.add(new Option(`${h}°`, h));
  if (selected != null && selected % HEADING_STEP_DEG) sel.add(new Option(`${Math.round(selected)}°`, selected));
  sel.value = selected == null ? '' : String(selected);
  return sel;
}

function renderPatrolStops() {
  const p = currentPatrol();
  const list = document.getElementById('patrol-stops');
  list.innerHTML = '';
  document.getElementById('patrol-stops-empty').hidden = p.stops.length > 0;
  p.stops.forEach((stop, i) => {
    const li = document.createElement('li');
    li.innerHTML = `
      <div class="stop-head">
        <span class="stop-num">${i + 1}</span>
        <span class="stop-wp"></span>
        <button data-act="up" title="Move earlier">↑</button>
        <button data-act="down" title="Move later">↓</button>
        <button data-act="remove" class="delete" title="Remove stop">✕</button>
      </div>
      <div class="stop-fields">
        <label>Face <span data-slot="heading"></span></label>
        <label>Hold <input data-k="dwell" type="number" min="0" max="600" step="1" /> s</label>
      </div>`;
    li.querySelector('.stop-wp').textContent = waypointLabel(stop.waypoint_id);
    const hs = headingSelect(stop.heading_deg);
    hs.addEventListener('change', () => {
      stop.heading_deg = hs.value === '' ? null : Number(hs.value);
      patrolEdited();
    });
    li.querySelector('[data-slot="heading"]').replaceWith(hs);
    const dwell = li.querySelector('[data-k="dwell"]');
    dwell.value = stop.dwell_s;
    dwell.addEventListener('change', () => {
      stop.dwell_s = Math.min(600, Math.max(0, parseFloat(dwell.value) || 0));
      dwell.value = stop.dwell_s;
      patrolEdited();
    });
    li.querySelector('[data-act="up"]').disabled = i === 0;
    li.querySelector('[data-act="down"]').disabled = i === p.stops.length - 1;
    li.querySelectorAll('[data-act]').forEach(b => b.addEventListener('click', () => {
      if (b.dataset.act === 'remove') {
        p.stops.splice(i, 1);
      } else {
        const j = b.dataset.act === 'up' ? i - 1 : i + 1;
        [p.stops[i], p.stops[j]] = [p.stops[j], p.stops[i]];
      }
      renderPatrolStops();
      patrolEdited();
    }));
    list.appendChild(li);
  });
  refreshPatrolOverlay();
  updatePatrolUI();
  state.map2d?.draw();
}

function patrolEdited() {
  setStatus('Patrol updated (unsaved)');
  refreshPatrolOverlay();
  state.map2d?.draw();
}

function addPatrolStop(waypointId, headingDeg = null) {
  currentPatrol().stops.push({ waypoint_id: waypointId, heading_deg: headingDeg, dwell_s: DEFAULT_DWELL_S });
  renderPatrolStops();
  patrolEdited();
}

// Show the selected patrol on the map while its tab is open, or while it's
// the one running (with the current stop highlighted).
function refreshPatrolOverlay() {
  const p = currentPatrol();
  const ns = state.live.navStatus;
  const run = state.live.enabled && ns?.state === 'patrolling' && ns.patrol?.name === p?.name
    ? ns.patrol : null;
  const tabOpen = document.getElementById('panel-patrol').classList.contains('active');
  state.patrolOverlay = p && (tabOpen || run)
    ? { stops: p.stops, loop: p.loop, current: run ? run.index : null } : null;
}

function updatePatrolUI() {
  const live = state.live.available;
  document.getElementById('patrol-run').hidden = !live;
  document.getElementById('patrol-stop').hidden = !live;
  document.getElementById('patrol-add-here').hidden = !live;
  const p = currentPatrol();
  if (!p) return;
  document.getElementById('patrol-run').disabled = !p.stops.length;
  document.getElementById('patrol-add-here').disabled = !(state.live.enabled && lookTargetId());
  const ns = state.live.navStatus, run = ns?.patrol;
  const mine = run && run.name === p.name && ns.state === 'patrolling';
  document.querySelectorAll('#patrol-stops li').forEach((li, i) =>
    li.classList.toggle('current', !!mine && run.index === i));
  const el = document.getElementById('patrol-status');
  if (!run) { el.textContent = ''; return; }
  const where = `"${run.name}" stop ${run.index + 1}/${run.total}${run.loop ? `, lap ${run.lap}` : ''}`;
  const doing = run.phase === 'dwelling' ? `holding ${run.dwell_s}s at ${waypointLabel(ns.destination)}`
    : `going to ${waypointLabel(ns.destination)}`
      + (ns.heading_deg != null ? `, to face ${Math.round(ns.heading_deg)}°` : '');
  el.textContent = ns.state === 'patrolling' ? `Running ${where}: ${doing}`
    : ns.state === 'reached' ? `Finished "${run.name}".`
    : ns.state === 'canceled' ? `Stopped at ${where}.`
    : `Failed at ${where}: ${ns.state}${ns.error ? ` (${ns.error})` : ''}`;
}

document.getElementById('patrol-select').addEventListener('change', e => {
  state.patrolIdx = Number(e.target.value);
  renderPatrols();
  state.map2d?.draw();
});
document.getElementById('patrol-new').addEventListener('click', () => {
  const patrols = state.annotations.patrols;
  patrols.push({ name: `patrol-${patrols.length + 1}`, stops: [], loop: false });
  state.patrolIdx = patrols.length - 1;
  renderPatrols();
  setPatrolAddMode(true);
  setStatus('New patrol: click waypoints on the map to add stops.');
});
document.getElementById('patrol-name').addEventListener('change', e => {
  const p = currentPatrol();
  p.name = e.target.value.trim() || p.name;
  renderPatrols();
  patrolEdited();
});
document.getElementById('patrol-loop').addEventListener('change', e => {
  currentPatrol().loop = e.target.checked;
  patrolEdited();
});
document.getElementById('patrol-add-mode').addEventListener('change', e => {
  setPatrolAddMode(e.target.checked);
  setStatus(e.target.checked ? 'Click waypoints on the map to add stops.' : '');
});
document.getElementById('patrol-add-here').addEventListener('click', () => {
  const id = lookTargetId();
  if (!id) return;
  // Prefer the heading Spot was told to face here; otherwise snap its actual
  // heading to the nearest dial step.
  const ns = state.live.navStatus, pose = state.live.pose;
  let h = ns?.destination === id ? ns.heading_deg : null;
  if (h == null && pose?.localized) {
    h = ((Math.round(pose.yaw * 180 / Math.PI / HEADING_STEP_DEG) * HEADING_STEP_DEG) % 360 + 360) % 360;
  }
  addPatrolStop(id, h);
});
document.getElementById('patrol-save').addEventListener('click', async () => {
  const r = await api('/annotations', {
    method: 'PUT', headers: { 'content-type': 'application/json' },
    body: JSON.stringify(state.annotations),
  });
  setStatus(r.ok ? 'Patrols saved' : 'Save failed');
});
document.getElementById('patrol-delete').addEventListener('click', () => {
  const p = currentPatrol();
  if (!p || !confirm(`Delete patrol "${p.name}"?`)) return;
  state.annotations.patrols.splice(state.patrolIdx, 1);
  state.patrolIdx -= 1;   // renderPatrols clamps
  renderPatrols();
  patrolEdited();
});
document.getElementById('patrol-run').addEventListener('click', async () => {
  const p = currentPatrol();
  if (!p?.stops.length) return;
  if (!state.live.enabled) document.getElementById('live-toggle').click();
  if (state.live.uploadedGraph !== state.graphName) {
    setStatus('Upload graph to robot first (Drive tab).');
    return;
  }
  const loop = p.loop ? ', repeating until STOP' : '';
  if (!confirm(`Run patrol "${p.name}" (${p.stops.length} stops${loop})?`)) return;
  setPatrolAddMode(false);
  const r = await api('/patrol', {
    method: 'POST', headers: { 'content-type': 'application/json' }, body: JSON.stringify(p),
  });
  const j = await r.json().catch(() => ({}));
  const detail = typeof j.detail === 'string' ? j.detail : JSON.stringify(j.detail ?? r.status);
  setStatus(r.ok ? `Running patrol "${p.name}"` : `Patrol failed to start: ${detail}`);
});
document.getElementById('patrol-stop').addEventListener('click', async () => {
  const r = await fetch('/api/stop', { method: 'POST' });
  setStatus(r.ok ? 'STOP sent' : 'STOP failed');
});

// --- Lease and software e-stop ---

async function pollControl() {
  try {
    state.live.estop = await fetch('/api/estop/state').then(r => r.json());
    updateEstopUI();
  } catch { /* ignore */ }
  try {
    const r = await fetch('/api/lease');
    state.live.lease = r.ok ? await r.json() : null;
  } catch { state.live.lease = null; }
  updateLeaseUI();
}

function updateLeaseUI() {
  const el = document.getElementById('lease-status');
  const l = state.live.lease;
  el.textContent = !l ? 'Lease: unknown'
    : l.ours ? 'Lease: this UI'
    : !l.holder || l.stale ? `Lease: free${l.holder ? ` (was ${l.holder})` : ''}`
    : `Lease: ${l.holder}`;
  el.style.color = l?.ours ? '#48bb78' : '#fc8181';
}

document.getElementById('lease-hijack').addEventListener('click', async () => {
  const l = state.live.lease;
  if (!l?.ours && !confirm(`Take the lease${l?.holder ? ` from ${l.holder}` : ''}? Whoever holds it loses control of Spot.`)) return;
  const r = await fetch('/api/lease/hijack', { method: 'POST' });
  const j = await r.json().catch(() => ({}));
  setStatus(r.ok ? 'Lease taken' : `Hijack failed: ${j.detail || r.status}`);
  pollControl();
});

const ESTOP_BADGE = {
  allowed:  { text: '● estop: armed',     color: '#48bb78' },
  settling: { text: '● estop: settling…', color: '#f6ad55' },
  cut:      { text: '● estop: CUT',       color: '#fc8181' },
  unknown:  { text: '● estop: unknown',   color: '#a0aec0' },
};

function updateEstopUI() {
  const badge = document.getElementById('estop-badge');
  const statusEl = document.getElementById('estop-status');
  const allowBtn = document.getElementById('estop-allow');
  const e = state.live.estop;
  if (!state.live.available || !e) {
    badge.hidden = true;
    statusEl.textContent = '—';
    allowBtn.disabled = true;
    return;
  }
  badge.hidden = false;
  const meta = ESTOP_BADGE[e.level] || ESTOP_BADGE.unknown;
  badge.textContent = meta.text;
  badge.style.color = meta.color;
  const otherNote = e.any_other_estopped ? ' (another endpoint also estopped)' : '';
  statusEl.innerHTML = `Endpoint: ${e.endpoint_registered ? 'registered' : 'NOT registered'}<br>Level: <b style="color:${meta.color}">${e.level}</b>${otherNote}`;
  // Enable Allow only if currently estopped from our endpoint.
  allowBtn.disabled = !(e.endpoint_registered && e.level !== 'allowed');
}

async function estopPost(path, label) {
  if (!confirm(`${label}? This will stop the robot.`)) return;
  const r = await fetch(path, { method: 'POST' });
  const j = await r.json().catch(() => ({}));
  setStatus(r.ok ? `${label} sent` : `${label} failed: ${j.detail || r.status}`);
  pollPose();
}

document.getElementById('estop-settle').addEventListener('click',
  () => estopPost('/api/estop/settle', 'Settle-then-cut'));
document.getElementById('estop-cut').addEventListener('click',
  () => estopPost('/api/estop/cut', 'Hard cut'));
document.getElementById('estop-allow').addEventListener('click', async () => {
  const r = await fetch('/api/estop/allow', { method: 'POST' });
  setStatus(r.ok ? 'E-stop re-armed' : 'Allow failed');
  pollPose();
});
document.getElementById('estop-popout').addEventListener('click', e => {
  e.preventDefault();
  window.open('/estop.html', 'graphnav_viz_estop',
    'width=340,height=320,popup=yes');
});

// --- Camera ---
// The <img> pulls an MJPEG multipart stream directly from the backend; there's
// no client-side polling loop. Clearing img.src closes the underlying connection,
// which is what stops the server asking Spot for images (see server.py). So it only
// runs while the box is ticked and the Drive tab is on screen.
let cameraSourcesLoaded = false;
let cameraStreaming = null;   // source of the open stream

const cameraWanted = () => document.getElementById('camera-on').checked
  && document.getElementById('panel-drive').classList.contains('active');

async function syncCamera() {
  document.getElementById('camera-controls').hidden = !document.getElementById('camera-on').checked;
  if (!cameraWanted()) { stopCameraStream(); return; }
  if (!cameraSourcesLoaded) await loadCameraSources();
  // The box or the tab may have changed during that fetch.
  if (cameraWanted()) startCameraStream(document.getElementById('camera-source').value);
}

async function loadCameraSources() {
  const status = document.getElementById('camera-status');
  status.textContent = 'Loading camera list…';
  try {
    const sources = await fetch('/api/cameras').then(r => r.json());
    const visual = sources.filter(s => s.image_type === 'visual');
    const sel = document.getElementById('camera-source');
    sel.innerHTML = '';
    (visual.length ? visual : sources).forEach(s => sel.add(new Option(s.name, s.name)));
    cameraSourcesLoaded = true;
    status.textContent = '';
  } catch (e) {
    status.textContent = `Failed to list cameras: ${e}`;
  }
}

function startCameraStream(source) {
  if (!source || source === cameraStreaming) return;
  cameraStreaming = source;
  document.getElementById('camera-status').textContent = '';
  document.getElementById('camera-view').src =
    `/api/camera_stream/${encodeURIComponent(source)}?t=${Date.now()}`;
}

function stopCameraStream() {
  cameraStreaming = null;
  document.getElementById('camera-view').removeAttribute('src');
}

document.getElementById('camera-on').addEventListener('change', syncCamera);
document.getElementById('camera-source').addEventListener('change', syncCamera);
document.getElementById('camera-view').addEventListener('error', () => {
  if (!cameraWanted()) return;
  cameraStreaming = null;
  document.getElementById('camera-status').textContent = 'Camera stream stopped. Untick and tick "Show camera" to retry.';
});

init().catch(err => {
  document.getElementById('status').textContent = `Load error: ${err}`;
  console.error(err);
});
