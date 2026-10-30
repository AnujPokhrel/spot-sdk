// 2D top-down canvas renderer with pan/zoom, click hit-test, polygon draw mode.

// Cool→warm colormap, returning [r,g,b] in 0..255, scaled down so additive
// blending against the dark background reads as cyan-yellow-red height tint.
function colormap(t) {
  t = Math.max(0, Math.min(1, t));
  // Three-stop ramp: blue → green → red, brightness 70.
  const r = 70 * Math.min(1, Math.max(0, 2 * t - 1));
  const g = 70 * Math.max(0, 1 - Math.abs(2 * t - 1));
  const b = 70 * Math.min(1, Math.max(0, 1 - 2 * t));
  return [r | 0, g | 0, b | 0];
}

function hexToRgba(hex, alpha) {
  const m = /^#?([0-9a-f]{6})$/i.exec(hex || '');
  if (!m) return `rgba(72,187,120,${alpha})`;            // fallback green
  const n = parseInt(m[1], 16);
  return `rgba(${(n >> 16) & 255},${(n >> 8) & 255},${n & 255},${alpha})`;
}

export const FIDUCIAL_COLORS = {
  localization:      '#f56565',  // red
  dock:              '#4fd1c5',  // teal
  dock_station:      '#38b2ac',  // dark teal
  mission_interrupt: '#ed8936',  // orange
  unknown:           '#a0aec0',  // gray
};

export class Map2D {
  constructor(canvas, graph, state, callbacks) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.graph = graph;
    this.state = state;        // shared annotations + selection
    this.cb = callbacks;       // { onSelectWaypoint, onSelectFiducial, onPolygonClosed }

    // World-frame center to canvas. Initial fit:
    const b = graph.bounds;
    this.center = { x: (b.min_x + b.max_x) / 2, y: (b.min_y + b.max_y) / 2 };
    this._fit();

    this.dragLast = null;
    this.polygonMode = null;     // null | { type: 'room' | 'nogo', vertices: [] }
    this.scans = null;           // Float32Array (3N) in seed frame

    this._handlers = {
      down: e => this._onDown(e),
      move: e => this._onMove(e),
      up: () => (this.dragLast = null),
      wheel: e => this._onWheel(e),
      dblclick: e => this._onDblClick(e),
      resize: () => { this._resize(); this.draw(); },
    };
    canvas.addEventListener('mousedown', this._handlers.down);
    canvas.addEventListener('mousemove', this._handlers.move);
    window.addEventListener('mouseup', this._handlers.up);
    canvas.addEventListener('wheel', this._handlers.wheel, { passive: false });
    canvas.addEventListener('dblclick', this._handlers.dblclick);
    window.addEventListener('resize', this._handlers.resize);
    this._resize();
  }

  destroy() {
    if (!this._handlers) return;
    this.canvas.removeEventListener('mousedown', this._handlers.down);
    this.canvas.removeEventListener('mousemove', this._handlers.move);
    window.removeEventListener('mouseup', this._handlers.up);
    this.canvas.removeEventListener('wheel', this._handlers.wheel);
    this.canvas.removeEventListener('dblclick', this._handlers.dblclick);
    window.removeEventListener('resize', this._handlers.resize);
    this._handlers = null;
    this.graph = null;
    this.scans = null;
    this._scanBitmap = null;
    this._scanBounds = null;
  }

  _fit() {
    const b = this.graph.bounds;
    const w = this.canvas.clientWidth || 800, h = this.canvas.clientHeight || 600;
    const dx = b.max_x - b.min_x, dy = b.max_y - b.min_y;
    this.scale = 0.85 * Math.min(w / Math.max(dx, 0.1), h / Math.max(dy, 0.1));
  }

  _resize() {
    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = this.canvas.clientWidth * dpr;
    this.canvas.height = this.canvas.clientHeight * dpr;
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  // World ↔ screen
  w2s(x, y) {
    const w = this.canvas.clientWidth, h = this.canvas.clientHeight;
    return [w / 2 + (x - this.center.x) * this.scale,
            h / 2 - (y - this.center.y) * this.scale];   // flip Y so +y is up
  }
  s2w(sx, sy) {
    const w = this.canvas.clientWidth, h = this.canvas.clientHeight;
    return [(sx - w / 2) / this.scale + this.center.x,
            -(sy - h / 2) / this.scale + this.center.y];
  }

  setScans(arr) {
    this.scans = arr;
    if (!arr || !arr.length) {
      this._scanBitmap = null;
      this._scanBounds = null;
      return;
    }
    this._buildScanBitmap(arr);
  }

  // Pre-rasterize the point cloud once into an offscreen canvas covering the
  // graph's world-frame bounds at a fixed pixel-per-meter resolution. draw()
  // then just blits this bitmap with the current pan/zoom transform — O(1)
  // per frame regardless of point count.
  _buildScanBitmap(arr) {
    const b = this.graph.bounds;
    const dx = Math.max(0.1, b.max_x - b.min_x);
    const dy = Math.max(0.1, b.max_y - b.min_y);
    // Target resolution: 60 px/m, but cap the largest axis so the bitmap
    // doesn't blow past ~8k pixels (or ~256 MB ImageData).
    const maxDim = 8192;
    const ppm = Math.min(60, maxDim / Math.max(dx, dy));
    const W = Math.max(1, Math.round(dx * ppm));
    const H = Math.max(1, Math.round(dy * ppm));
    const offscreen = (typeof OffscreenCanvas !== 'undefined')
      ? new OffscreenCanvas(W, H)
      : Object.assign(document.createElement('canvas'), { width: W, height: H });
    const octx = offscreen.getContext('2d');
    const img = octx.createImageData(W, H);
    const data = img.data;
    const zmin = b.min_z, zspan = Math.max(1e-3, b.max_z - b.min_z);
    const minX = b.min_x, maxY = b.max_y;
    for (let i = 0; i < arr.length; i += 3) {
      const x = arr[i], y = arr[i + 1], z = arr[i + 2];
      const px = ((x - minX) * ppm) | 0;
      const py = ((maxY - y) * ppm) | 0;       // flip Y so +y is up in world
      if (px < 0 || px >= W || py < 0 || py >= H) continue;
      const t = (z - zmin) / zspan;
      const [r, g, bl] = colormap(t);
      const idx = (py * W + px) * 4;
      data[idx]     = Math.min(255, data[idx]     + r);
      data[idx + 1] = Math.min(255, data[idx + 1] + g);
      data[idx + 2] = Math.min(255, data[idx + 2] + bl);
      data[idx + 3] = 255;
    }
    octx.putImageData(img, 0, 0);
    this._scanBitmap = offscreen;
    this._scanBounds = { minX, maxY, W, H, ppm };
  }

  startPolygon(type) {
    this.polygonMode = { type, vertices: [] };
    this.draw();
  }
  cancelPolygon() {
    this.polygonMode = null;
    this.draw();
  }

  _onDown(e) {
    const rect = this.canvas.getBoundingClientRect();
    const sx = e.clientX - rect.left, sy = e.clientY - rect.top;

    if (this.polygonMode) {
      const [wx, wy] = this.s2w(sx, sy);
      this.polygonMode.vertices.push([wx, wy]);
      this.draw();
      return;
    }

    if (e.button === 0) {
      // In schematic mode, waypoints/fiducials are hidden, so skip hit-test
      // and fall through to pan.
      if (this.state.schematic) { this.dragLast = { sx, sy }; return; }
      // Hit-test waypoints (radius 8 px) and fiducials (10 px)
      for (const f of this.graph.fiducials) {
        const [fx, fy] = this.w2s(f.x, f.y);
        if (Math.hypot(fx - sx, fy - sy) < 12) {
          this.cb.onSelectFiducial?.(f);
          return;
        }
      }
      for (const wp of this.graph.waypoints) {
        const [wx, wy] = this.w2s(wp.x, wp.y);
        if (Math.hypot(wx - sx, wy - sy) < 9) {
          this.cb.onSelectWaypoint?.(wp);
          return;
        }
      }
      this.dragLast = { sx, sy };
    }
  }
  _onMove(e) {
    if (!this.dragLast) return;
    const rect = this.canvas.getBoundingClientRect();
    const sx = e.clientX - rect.left, sy = e.clientY - rect.top;
    this.center.x -= (sx - this.dragLast.sx) / this.scale;
    this.center.y += (sy - this.dragLast.sy) / this.scale;
    this.dragLast = { sx, sy };
    this.draw();
  }
  _onWheel(e) {
    e.preventDefault();
    const rect = this.canvas.getBoundingClientRect();
    const sx = e.clientX - rect.left, sy = e.clientY - rect.top;
    const [wxBefore, wyBefore] = this.s2w(sx, sy);
    this.scale *= e.deltaY < 0 ? 1.15 : (1 / 1.15);
    const [wxAfter, wyAfter] = this.s2w(sx, sy);
    this.center.x += wxBefore - wxAfter;
    this.center.y += wyBefore - wyAfter;
    this.draw();
  }
  _onDblClick(e) {
    if (this.polygonMode && this.polygonMode.vertices.length >= 3) {
      const closed = this.polygonMode;
      this.polygonMode = null;
      this.cb.onPolygonClosed?.(closed.type, closed.vertices);
      this.draw();
    }
  }

  draw() {
    const ctx = this.ctx;
    const w = this.canvas.clientWidth, h = this.canvas.clientHeight;
    ctx.clearRect(0, 0, w, h);

    const schematic = !!this.state.schematic;

    // Scans (point-cloud underlay), drawn from a pre-rasterized offscreen
    // bitmap built once in setScans(). Place by mapping the bitmap's
    // world-frame bounds through the current pan/zoom.
    if (!schematic && this._scanBitmap && this._scanBounds) {
      const { minX, maxY, W, H, ppm } = this._scanBounds;
      const [sx0, sy0] = this.w2s(minX, maxY);  // bitmap's top-left in world
      const dw = (W / ppm) * this.scale;
      const dh = (H / ppm) * this.scale;
      ctx.drawImage(this._scanBitmap, sx0, sy0, dw, dh);
    }

    // Rooms — per-room color (defaults to green when unset).
    ctx.lineWidth = 1.5;
    for (const r of this.state.annotations.rooms) {
      const c = r.color || '#48bb78';
      ctx.fillStyle = hexToRgba(c, 0.18);
      ctx.strokeStyle = hexToRgba(c, 0.7);
      this._fillPoly(r.polygon_seed);
      this._labelPoly(r.polygon_seed, r.name, c);
    }
    // No-go (always red — clear danger semantics)
    ctx.fillStyle = 'rgba(229, 62, 62, 0.15)';
    ctx.strokeStyle = 'rgba(229, 62, 62, 0.7)';
    for (const z of this.state.annotations.no_go_zones) {
      this._fillPoly(z.polygon_seed);
      this._labelPoly(z.polygon_seed, z.name, '#fc8181');
    }

    if (!schematic) {
      // Edges
      ctx.strokeStyle = 'rgba(160, 174, 192, 0.45)';
      ctx.lineWidth = 1;
      const wpById = Object.fromEntries(this.graph.waypoints.map(w => [w.id, w]));
      ctx.beginPath();
      for (const e of this.graph.edges) {
        const a = wpById[e.from], b = wpById[e.to];
        if (!a || !b) continue;
        const [ax, ay] = this.w2s(a.x, a.y);
        const [bx, by] = this.w2s(b.x, b.y);
        ctx.moveTo(ax, ay); ctx.lineTo(bx, by);
      }
      ctx.stroke();

      // Waypoints
      for (const wp of this.graph.waypoints) {
        const [x, y] = this.w2s(wp.x, wp.y);
        const poi = this.state.annotations.waypoint_pois[wp.id];
        const tagged = poi && (poi.name || poi.tags?.length);
        ctx.fillStyle = (this.state.selectedWaypointId === wp.id) ? '#f6ad55'
          : tagged ? '#63b3ed' : '#a0aec0';
        ctx.beginPath();
        ctx.arc(x, y, 5, 0, Math.PI * 2);
        ctx.fill();
      }

      // Fiducials, colored by category. Doors (those in annotations.doors) get
      // an outlined ring on top of the localization color.
      ctx.font = '11px ui-monospace, monospace';
      const doorTagIds = new Set((this.state.annotations.doors || []).map(d => d.tag_id));
      for (const f of this.graph.fiducials) {
        const [x, y] = this.w2s(f.x, f.y);
        ctx.fillStyle = FIDUCIAL_COLORS[f.category] || FIDUCIAL_COLORS.unknown;
        ctx.fillRect(x - 6, y - 6, 12, 12);
        if (doorTagIds.has(f.tag_id)) {
          ctx.strokeStyle = '#f6e05e';
          ctx.lineWidth = 2;
          ctx.strokeRect(x - 9, y - 9, 18, 18);
        }
        ctx.fillStyle = '#fff';
        const tag = `#${f.tag_id}`;
        const cat = doorTagIds.has(f.tag_id) ? 'door' : (f.category || 'unknown');
        ctx.fillText(`${tag} ${cat}`, x + 11, y - 4);
      }
    }

    // Patrol: dashed line in stop order (straight segments; GraphNav still
    // follows the recorded edges), an arrow per stop heading, and numbered
    // badges. A waypoint visited twice gets one badge like "2,5".
    const patrol = this.state.patrolOverlay;
    if (patrol?.stops.length) {
      const PURPLE = '#b794f4';
      const byId = new Map(this.graph.waypoints.map(w => [w.id, w]));
      const stops = patrol.stops
        .map((st, i) => ({ ...st, i, wp: byId.get(st.waypoint_id) }))
        .filter(st => st.wp);
      ctx.save();
      ctx.strokeStyle = ctx.fillStyle = PURPLE;
      ctx.lineWidth = 2;
      ctx.setLineDash([6, 5]);
      ctx.beginPath();
      stops.forEach((st, k) => {
        const [x, y] = this.w2s(st.wp.x, st.wp.y);
        if (k) ctx.lineTo(x, y); else ctx.moveTo(x, y);
      });
      if (patrol.loop && stops.length > 2) ctx.closePath();
      ctx.stroke();
      ctx.setLineDash([]);
      for (const st of stops) {
        if (st.heading_deg == null) continue;
        const [x, y] = this.w2s(st.wp.x, st.wp.y);
        ctx.save();
        ctx.translate(x, y);
        ctx.rotate(-st.heading_deg * Math.PI / 180);
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(24, 0);
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(31, 0);
        ctx.lineTo(23, 4.5);
        ctx.lineTo(23, -4.5);
        ctx.closePath();
        ctx.fill();
        ctx.restore();
      }
      const badges = new Map();   // waypoint id -> stop indices
      for (const st of stops) badges.set(st.waypoint_id, [...(badges.get(st.waypoint_id) || []), st.i]);
      ctx.font = 'bold 11px ui-sans-serif, system-ui';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      for (const [id, idxs] of badges) {
        const wp = byId.get(id);
        const [x, y] = this.w2s(wp.x, wp.y);
        const text = idxs.map(i => i + 1).join(',');
        const w = Math.max(18, ctx.measureText(text).width + 10);
        ctx.beginPath();
        if (ctx.roundRect) ctx.roundRect(x - w / 2, y - 9, w, 18, 9);
        else ctx.rect(x - w / 2, y - 9, w, 18);
        ctx.fillStyle = PURPLE;
        ctx.fill();
        if (idxs.includes(patrol.current)) {
          ctx.strokeStyle = '#00e5ff';
          ctx.lineWidth = 2.5;
          ctx.stroke();
        }
        ctx.fillStyle = '#1a202c';
        ctx.fillText(text, x, y + 0.5);
      }
      ctx.restore();
    }

    // Look-around: heading ticks around the waypoint Spot would turn at, and
    // an arrow for the commanded heading (map frame, same convention as yaw).
    const look = this.state.live?.enabled ? this.state.live.look : null;
    const lookWp = look && this.graph.waypoints.find(w => w.id === look.waypointId);
    if (lookWp) {
      const [cx, cy] = this.w2s(lookWp.x, lookWp.y);
      ctx.save();
      ctx.translate(cx, cy);
      ctx.strokeStyle = 'rgba(246, 173, 85, 0.55)';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.arc(0, 0, 22, 0, Math.PI * 2);
      for (let h = 0; h < 360; h += look.stepDeg) {
        const a = h * Math.PI / 180;
        ctx.moveTo(22 * Math.cos(a), -22 * Math.sin(a));
        ctx.lineTo(28 * Math.cos(a), -28 * Math.sin(a));
      }
      ctx.stroke();
      if (look.headingDeg != null) {
        ctx.rotate(-look.headingDeg * Math.PI / 180);
        ctx.strokeStyle = ctx.fillStyle = '#f6ad55';
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(36, 0);
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(44, 0);
        ctx.lineTo(34, 5);
        ctx.lineTo(34, -5);
        ctx.closePath();
        ctx.fill();
      }
      ctx.restore();
    }

    // Live robot pose (drawn on top of everything else).
    const pose = this.state.live?.pose;
    if (this.state.live?.enabled && pose) {
      const matches = pose.uploaded_graph === this.state.graphName;
      const fill = (pose.localized && matches) ? '#00e5ff' : '#a0aec0';
      const [px, py] = this.w2s(pose.x, pose.y);
      ctx.save();
      ctx.translate(px, py);
      // Canvas Y is flipped vs. world Y, so negate yaw for screen rotation.
      ctx.rotate(-pose.yaw);
      ctx.fillStyle = fill;
      ctx.strokeStyle = '#0f1116';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(14, 0);
      ctx.lineTo(-9, 7);
      ctx.lineTo(-9, -7);
      ctx.closePath();
      ctx.fill();
      ctx.stroke();
      ctx.restore();
    }

    // In-progress polygon
    if (this.polygonMode) {
      ctx.strokeStyle = this.polygonMode.type === 'room' ? '#48bb78' : '#fc8181';
      ctx.fillStyle = this.polygonMode.type === 'room'
        ? 'rgba(72,187,120,0.1)' : 'rgba(252,129,129,0.1)';
      ctx.setLineDash([5, 4]);
      this._fillPoly(this.polygonMode.vertices, true);
      ctx.setLineDash([]);
      ctx.fillStyle = '#fff';
      ctx.fillText(`drawing ${this.polygonMode.type} (double-click to finish)`, 12, 22);
    }
  }

  _fillPoly(pts, openOk = false) {
    if (pts.length < 2) return;
    const ctx = this.ctx;
    ctx.beginPath();
    pts.forEach(([x, y], i) => {
      const [sx, sy] = this.w2s(x, y);
      if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
    });
    if (!openOk || pts.length >= 3) ctx.closePath();
    ctx.fill();
    ctx.stroke();
  }
  _labelPoly(pts, label, color) {
    if (!pts.length) return;
    const cx = pts.reduce((s, p) => s + p[0], 0) / pts.length;
    const cy = pts.reduce((s, p) => s + p[1], 0) / pts.length;
    const [sx, sy] = this.w2s(cx, cy);
    this.ctx.fillStyle = color;
    this.ctx.font = 'bold 12px ui-sans-serif, system-ui';
    this.ctx.fillText(label, sx, sy);
  }
}
