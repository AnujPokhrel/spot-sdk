// Top-level controller: graph picker, fetch graph + annotations, save/export.
import { Map2D } from './map2d.js';

const state = {
  graphName: null,
  graph: null,
  annotations: { doors: [], waypoint_pois: {}, rooms: [], no_go_zones: [] },
  selectedWaypointId: null,
  view: '2d',
  map2d: null, map3d: null,
  scans: null,        // Float32Array (Nx3) in seed frame, lazy-loaded
  scansVisible: false,
  schematic: false,   // 2D schematic mode: hide scans/edges/waypoints/fiducials
  loadToken: 0,       // increments per loadGraph; late responses bail if mismatched
  live: {
    available: false,   // server has --live session
    enabled: false,     // user toggled it on
    pose: null,         // last poll result
    mode: 'select',     // 'select' | 'drive' — drive sends nav commands on waypoint click
    uploadedGraph: null,
    pollTimer: null,
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
    if (state.live.available) {
      document.getElementById('live-toggle').hidden = false;
      document.getElementById('tab-drive').hidden = false;
      document.getElementById('tab-cameras').hidden = false;
      // Independent of the Live toggle, keep the estop state fresh so the
      // operator always knows if the robot is armed/cut.
      setInterval(async () => {
        try {
          state.live.estop = await fetch('/api/estop/state').then(r => r.json());
          updateEstopUI();
        } catch { /* ignore */ }
      }, 1000);
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
    onPolygonClosed: (type, vertices) => onPolygonClosed(type, vertices),
  });
  state.map2d.draw();

  // Reset 3D container
  const c3 = document.getElementById('canvas3d');
  c3.innerHTML = '';

  // If currently in 3D, rebuild it now
  if (state.view === '3d') await ensure3D(token);
  if (state.loadToken !== token) return;

  // Reset waypoint form
  document.getElementById('wp-empty').hidden = false;
  document.getElementById('wp-form').hidden = true;

  renderDoors();
  renderRooms();
  renderNoGo();
}

// --- Tabs ---
document.querySelectorAll('#tabs .tab').forEach(tab => {
  tab.addEventListener('click', () => {
    document.querySelectorAll('#tabs .tab').forEach(t => t.classList.remove('active'));
    document.querySelectorAll('.panel').forEach(p => p.classList.remove('active'));
    tab.classList.add('active');
    document.getElementById(`panel-${tab.dataset.tab}`).classList.add('active');
    if (tab.dataset.tab === 'cameras') {
      loadCameraSources();
    } else {
      stopCameraStream();
    }
  });
});

// --- Waypoint panel ---
function selectWaypoint(wp) {
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
  document.querySelector('[data-tab="waypoint"]').click();
  document.getElementById('wp-empty').hidden = true;
  document.getElementById('wp-form').hidden = false;
  document.getElementById('wp-id').value = wp.id;
  const poi = state.annotations.waypoint_pois[wp.id] || { name: wp.name, tags: [], notes: '' };
  document.getElementById('wp-name').value = poi.name || wp.name;
  document.getElementById('wp-tags').value = (poi.tags || []).join(', ');
  document.getElementById('wp-notes').value = poi.notes || '';
  state.map2d?.draw();
}
document.getElementById('wp-apply').addEventListener('click', () => {
  const id = document.getElementById('wp-id').value;
  state.annotations.waypoint_pois[id] = {
    name: document.getElementById('wp-name').value.trim(),
    tags: document.getElementById('wp-tags').value.split(',').map(s => s.trim()).filter(Boolean),
    notes: document.getElementById('wp-notes').value,
  };
  setStatus('Updated (unsaved)');
  state.map2d?.draw();
});

// --- Doors panel ---
const CATEGORY_LABELS = {
  localization:      'Localization (1–299)',
  dock:              'Spot Dock (520–549)',
  dock_station:      'Spot Dock — Spot Station (580–584)',
  mission_interrupt: 'Mission interrupt (585–586)',
  unknown:           'Unknown / out-of-range',
};

function selectFiducial(f) {
  document.querySelector('[data-tab="doors"]').click();
  const el = document.getElementById(`fid-card-${f.tag_id}`);
  el?.scrollIntoView({ behavior: 'smooth', block: 'center' });
  el?.classList.add('focus');
  setTimeout(() => el?.classList.remove('focus'), 1500);
}

function renderDoors() {
  const list = document.getElementById('doors-list');
  list.innerHTML = '';
  if (!state.graph.fiducials.length) {
    list.innerHTML = '<div class="muted">No fiducials in this graph.</div>';
    return;
  }

  const byCategory = {};
  for (const f of state.graph.fiducials) {
    (byCategory[f.category] = byCategory[f.category] || []).push(f);
  }

  // Render localization first (the only category that can be a door),
  // then docks / mission interrupts / unknown as info-only.
  const order = ['localization', 'dock', 'dock_station', 'mission_interrupt', 'unknown'];
  for (const cat of order) {
    const fids = byCategory[cat];
    if (!fids?.length) continue;
    const header = document.createElement('h3');
    header.textContent = CATEGORY_LABELS[cat];
    header.style.cssText = 'font-size:12px; color:#a0aec0; text-transform:uppercase; letter-spacing:0.05em; margin:1rem 0 0.5rem;';
    list.appendChild(header);

    for (const f of fids) {
      list.appendChild(cat === 'localization' ? doorCard(f) : infoCard(f));
    }
  }
}

function infoCard(f) {
  const card = document.createElement('div');
  card.className = 'card';
  card.id = `fid-card-${f.tag_id}`;
  card.innerHTML = `
    <h3>#${f.tag_id} <span class="muted">@ (${f.x.toFixed(2)}, ${f.y.toFixed(2)})</span></h3>
    <div class="muted">Reserved by Boston Dynamics — not a door.</div>
  `;
  return card;
}

function doorCard(f) {
  const card = document.createElement('div');
  card.className = 'card';
  card.id = `fid-card-${f.tag_id}`;
  const existing = state.annotations.doors.find(d => d.tag_id === f.tag_id);

  if (!existing) {
    card.innerHTML = `
      <h3>#${f.tag_id} <span class="muted">@ (${f.x.toFixed(2)}, ${f.y.toFixed(2)})</span></h3>
      <div class="muted">Localization tag — not marked as a door.</div>
      <button>Mark as door</button>
    `;
    card.querySelector('button').addEventListener('click', () => {
      state.annotations.doors.push({
        tag_id: f.tag_id, name: `door_${f.tag_id}`,
        tag_to_handle: [0, 0, 0.12], handle_type: 'lever',
        swing: 'pull', hinge: 'right',
        nav_waypoint: nearestWaypointName(f), approach_standoff_m: 1.0,
      });
      renderDoors();
      state.map2d?.draw();
      setStatus(`#${f.tag_id} marked as door (unsaved)`);
    });
    return card;
  }

  card.innerHTML = `
    <h3>#${f.tag_id} <span class="muted">@ (${f.x.toFixed(2)}, ${f.y.toFixed(2)})</span> · door</h3>
    <label>Name <input data-k="name" value="${existing.name}"></label>
    <label>Tag→handle [x y z] (m) <input data-k="tag_to_handle" value="${existing.tag_to_handle.join(' ')}"></label>
    <div class="row">
      <label>Handle <select data-k="handle_type">
        <option ${existing.handle_type === 'lever' ? 'selected' : ''}>lever</option>
        <option ${existing.handle_type === 'push_bar' ? 'selected' : ''}>push_bar</option>
      </select></label>
      <label>Swing <select data-k="swing">
        <option ${existing.swing === 'pull' ? 'selected' : ''}>pull</option>
        <option ${existing.swing === 'push' ? 'selected' : ''}>push</option>
      </select></label>
      <label>Hinge <select data-k="hinge">
        <option ${existing.hinge === 'right' ? 'selected' : ''}>right</option>
        <option ${existing.hinge === 'left' ? 'selected' : ''}>left</option>
      </select></label>
    </div>
    <label>Nav waypoint <input data-k="nav_waypoint" value="${existing.nav_waypoint || ''}"></label>
    <label>Standoff (m) <input data-k="approach_standoff_m" type="number" step="0.1" value="${existing.approach_standoff_m}"></label>
    <div class="row">
      <button data-action="apply">Apply</button>
      <button data-action="remove" class="delete">Unmark as door</button>
    </div>
  `;
  card.querySelector('[data-action="apply"]').addEventListener('click', () => {
    const get = k => card.querySelector(`[data-k="${k}"]`).value;
    const door = {
      tag_id: f.tag_id,
      name: get('name').trim(),
      tag_to_handle: get('tag_to_handle').split(/\s+/).map(Number),
      handle_type: get('handle_type'),
      swing: get('swing'),
      hinge: get('hinge'),
      nav_waypoint: get('nav_waypoint').trim() || null,
      approach_standoff_m: parseFloat(get('approach_standoff_m')),
    };
    state.annotations.doors = state.annotations.doors.filter(d => d.tag_id !== f.tag_id);
    state.annotations.doors.push(door);
    setStatus(`Updated door #${f.tag_id} (unsaved)`);
  });
  card.querySelector('[data-action="remove"]').addEventListener('click', () => {
    state.annotations.doors = state.annotations.doors.filter(d => d.tag_id !== f.tag_id);
    renderDoors();
    state.map2d?.draw();
    setStatus(`Removed door annotation for #${f.tag_id} (unsaved)`);
  });
  return card;
}
function nearestWaypointName(f) {
  let best = null, bestD = Infinity;
  for (const wp of state.graph.waypoints) {
    const d = Math.hypot(wp.x - f.x, wp.y - f.y);
    if (d < bestD) { bestD = d; best = wp; }
  }
  if (!best) return '';
  const poi = state.annotations.waypoint_pois[best.id];
  return (poi?.name) || best.name || '';
}

// --- Rooms / no-go ---
function renderRooms() { renderPolygonList('rooms-list', state.annotations.rooms, 'rooms'); }
function renderNoGo() { renderPolygonList('nogo-list', state.annotations.no_go_zones, 'no_go_zones'); }
function renderPolygonList(elId, list, key) {
  const el = document.getElementById(elId);
  el.innerHTML = '';
  list.forEach((p, idx) => {
    const card = document.createElement('div');
    card.className = 'card';
    const colorInput = key === 'rooms'
      ? `<label>Color <input data-k="color" type="color" value="${p.color || '#48bb78'}"></label>`
      : '';
    card.innerHTML = `
      <h3>${p.name}</h3>
      <label>Name <input data-k="name" value="${p.name}"></label>
      ${colorInput}
      <div class="muted">${p.polygon_seed.length} vertices${p.waypoints ? ` · ${p.waypoints.length} waypoints` : ''}</div>
      <button class="delete">Delete</button>
    `;
    card.querySelector('[data-k="name"]').addEventListener('change', e => {
      list[idx].name = e.target.value;
      state.map2d?.draw();
    });
    card.querySelector('[data-k="color"]')?.addEventListener('input', e => {
      list[idx].color = e.target.value;
      state.map2d?.draw();
    });
    card.querySelector('.delete').addEventListener('click', () => {
      state.annotations[key].splice(idx, 1);
      renderPolygonList(elId, state.annotations[key], key);
      state.map2d?.draw();
    });
    el.appendChild(card);
  });
}
document.getElementById('room-new').addEventListener('click', () => {
  state.map2d.startPolygon('room');
  setStatus('Click on map to add vertices, double-click to finish.');
});
document.getElementById('nogo-new').addEventListener('click', () => {
  state.map2d.startPolygon('nogo');
  setStatus('Click on map to add vertices, double-click to finish.');
});
function onPolygonClosed(type, vertices) {
  const name = prompt(`Name for this ${type}:`) || `${type}-${Date.now()}`;
  if (type === 'room') {
    const inside = state.graph.waypoints
      .filter(wp => pointInPoly(wp.x, wp.y, vertices))
      .map(wp => wp.id);
    state.annotations.rooms.push({ name, polygon_seed: vertices, waypoints: inside });
    renderRooms();
  } else {
    state.annotations.no_go_zones.push({ name, polygon_seed: vertices });
    renderNoGo();
  }
  state.map2d?.draw();
  setStatus(`Added ${type} (unsaved)`);
}
function pointInPoly(x, y, poly) {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const [xi, yi] = poly[i], [xj, yj] = poly[j];
    if (((yi > y) !== (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi)) inside = !inside;
  }
  return inside;
}

// --- Top bar ---
document.getElementById('save-btn').addEventListener('click', async () => {
  const r = await api('/annotations', {
    method: 'PUT', headers: { 'content-type': 'application/json' },
    body: JSON.stringify(state.annotations),
  });
  setStatus(r.ok ? 'Saved annotations.json' : 'Save failed');
});
document.getElementById('export-doors-btn').addEventListener('click', async () => {
  // Save first so the server has up-to-date annotations to export.
  await api('/annotations', {
    method: 'PUT', headers: { 'content-type': 'application/json' },
    body: JSON.stringify(state.annotations),
  });
  const r = await api('/export/doors', { method: 'POST' });
  if (!r.ok) { setStatus('Export failed'); return; }
  const j = await r.json();
  setStatus(`Wrote ${j.path} (${j.count} doors)`);
});

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

document.getElementById('schematic-toggle').addEventListener('click', () => {
  state.schematic = !state.schematic;
  document.getElementById('schematic-toggle').classList.toggle('active', state.schematic);
  state.map2d?.draw();
});

document.getElementById('view-toggle').addEventListener('click', async () => {
  const c2 = document.getElementById('canvas2d'), c3 = document.getElementById('canvas3d');
  const schBtn = document.getElementById('schematic-toggle');
  if (state.view === '2d') {
    c2.style.display = 'none'; c3.style.display = 'block';
    state.view = '3d';
    document.getElementById('view-toggle').textContent = 'Switch to 2D';
    schBtn.disabled = true;
    await ensure3D();
  } else {
    c3.style.display = 'none'; c2.style.display = 'block';
    state.view = '2d';
    document.getElementById('view-toggle').textContent = 'Switch to 3D';
    schBtn.disabled = false;
    state.map2d?.draw();
  }
});

// --- Live mode ---

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
    state.live.navStatus ? `Nav: ${state.live.navStatus.state}${state.live.navStatus.error ? ` (${state.live.navStatus.error})` : ''}` : '',
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
  updateEstopUI();
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
    state.map2d?.draw();
    state.map3d?.setLivePose?.(null);
    stopCameraStream();
  }
});

document.getElementById('drive-mode').addEventListener('change', e => {
  state.live.mode = e.target.checked ? 'drive' : 'select';
  // Click-to-navigate requires Live polling to be on (it's what gates the
  // nav request and drives pose/estop status) — checking this box is
  // useless on its own if the user forgot to flip the header toggle, so
  // turn it on for them instead of silently no-op'ing on waypoint clicks.
  if (e.target.checked && !state.live.enabled) {
    document.getElementById('live-toggle').click();
  }
  setStatus(state.live.mode === 'drive'
    ? 'Drive mode: click a waypoint to send Spot there.'
    : 'Select mode.');
});

document.getElementById('drive-power-on').addEventListener('click', async () => {
  setStatus('Powering on motors…');
  const r = await fetch('/api/power_on', { method: 'POST' });
  const j = await r.json().catch(() => ({}));
  setStatus(r.ok ? `Powered on: ${j.powered_on}` : `Power-on failed: ${j.detail || r.status}`);
});

document.getElementById('drive-stand').addEventListener('click', async () => {
  setStatus('Standing…');
  try {
    const r = await fetch('/api/stand', { method: 'POST' });
    const j = await r.json().catch(() => ({}));
    setStatus(r.ok ? 'Standing' : `Stand failed: ${j.detail || r.status}`);
  } catch (e) {
    setStatus('Network error while trying to stand.');
  }
});

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

document.getElementById('drive-stop').addEventListener('click', async () => {
  const r = await fetch('/api/stop', { method: 'POST' });
  setStatus(r.ok ? 'STOP sent' : 'STOP failed');
});

// --- Software e-stop ---

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

// --- Cameras ---
// The <img> pulls an MJPEG multipart stream directly from the backend; there's
// no client-side polling loop. Setting img.src to '' closes the underlying
// connection, which is what stops the server generator (see server.py).
let cameraSourcesLoaded = false;

async function loadCameraSources() {
  const empty = document.getElementById('camera-empty');
  const controls = document.getElementById('camera-controls');
  if (!state.live.available) {
    empty.hidden = false;
    controls.hidden = true;
    return;
  }
  empty.hidden = true;
  controls.hidden = false;
  if (cameraSourcesLoaded) {
    startCameraStream(document.getElementById('camera-source').value);
    return;
  }
  document.getElementById('camera-status').textContent = 'Loading camera list…';
  try {
    const sources = await fetch('/api/cameras').then(r => r.json());
    const visual = sources.filter(s => s.image_type === 'visual');
    const sel = document.getElementById('camera-source');
    sel.innerHTML = '';
    (visual.length ? visual : sources).forEach(s => {
      const opt = document.createElement('option');
      opt.value = s.name;
      opt.textContent = s.name;
      sel.appendChild(opt);
    });
    cameraSourcesLoaded = true;
    document.getElementById('camera-status').textContent = '';
    if (sel.options.length) startCameraStream(sel.value);
  } catch (e) {
    document.getElementById('camera-status').textContent = `Failed to list cameras: ${e}`;
  }
}

function startCameraStream(source) {
  if (!source) return;
  document.getElementById('camera-view').src =
    `/api/camera_stream/${encodeURIComponent(source)}?t=${Date.now()}`;
}

function stopCameraStream() {
  document.getElementById('camera-view').src = '';
}

document.getElementById('camera-source').addEventListener('change', e => {
  startCameraStream(e.target.value);
});

init().catch(err => {
  document.getElementById('status').textContent = `Load error: ${err}`;
  console.error(err);
});
