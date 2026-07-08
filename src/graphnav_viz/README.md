# graphnav_viz

A browser-based annotation app for Boston Dynamics **GraphNav** maps.

![2D top-down view](assets/2D_graphnav_noscan.png)

It reads a graph that was already recorded (with `graph_nav_command_line` or
Autowalk), renders it as an interactive 2D / 3D map, and lets you enrich it
with metadata that downstream tasks consume:

- **Doors** — per-fiducial form (handle type, swing, hinge, tag→handle offset,
  nearest waypoint). Exports to a YAML file consumed by a fiducial-driven
  door-opening client.
- **Waypoint POIs** — rename, tag, free-text notes per waypoint.
- **Rooms** — polygons named on the floorplan that group waypoints. Lets a
  planner answer "go to the kitchen" by picking any waypoint inside.
- **No-go zones** — polygons rendered as overlay, exported for planners.

Annotations live in a sidecar `annotations.json` next to the graph; the
recorded graph itself is never modified.

## Install

```bash
pip install fastapi uvicorn pyyaml pydantic numpy bosdyn-client
# Only needed if you plan to run with --live (auto-loads .env credentials):
pip install python-dotenv
```

(Or pin to your SDK version: `bosdyn-client==5.1.4`.)

### Tools & libraries

| Layer | What we use | Why |
|---|---|---|
| Web server | **FastAPI** + **Uvicorn** | minimal-boilerplate REST endpoints, GZip-compressed responses for large point-cloud payloads |
| Schema validation | **Pydantic** | typed `Annotations` model + automatic JSON (de)serialization for the sidecar |
| Config | **PyYAML** | optional `graphnav_viz.yaml` for graphs root + server settings |
| Graph parsing | **bosdyn-client** (`bosdyn.api.graph_nav.map_pb2`, `math_helpers`) | reads recorded graph protos; no VTK / ROS dependency |
| Numerics | **NumPy** | point-cloud transforms into the seed frame |
| 2D rendering | plain `<canvas>` + an offscreen pre-rasterized bitmap | scales to ~3 M points without per-frame work |
| 3D rendering | **three.js** (loaded from a CDN via import-map; no build step) | OrbitControls, Points, InstancedMesh |
| Live mode (opt-in) | **bosdyn-client** GraphNav / Lease / RobotCommand clients, **python-dotenv** | online robot pose + click-to-navigate |

## Quick start

From the repo root:

```bash
# Offline (annotation only, no robot needed)
PYTHONPATH=src python -m graphnav_viz.server
# open http://127.0.0.1:8000

# Live (with a Spot connected — pose overlay + click-to-navigate)
PYTHONPATH=src python -m graphnav_viz.server --live $SPOT_IP
```

The default config looks in `./recorded_graphs/` for any directory containing
a `graph` proto file (walks up to 2 levels deep).

CLI flags:

| Flag | Default | Purpose |
|---|---|---|
| `--config <path>` | `./graphnav_viz.yaml` if present | YAML config file (see below) |
| `--graphs <dir>` | from config | parent dir to scan, OR a single graph dir |
| `--host` / `--port` | `127.0.0.1` / `8000` | uvicorn bind |
| `--live <SPOT_IP>` | (off) | connect to a Spot and enable Drive features |

## Config file

Server settings and graph locations can be written to
`graphnav_viz.yaml` (loaded automatically from the cwd, or via `--config`):

```yaml
# Parent directory scanned for graphs. null disables scanning.
graphs_root: recorded_graphs

# Explicit graphs (override discovered ones on name collision). Useful for
# graphs outside graphs_root, e.g. a network share.
graphs:
  - name: warehouse
    path: /mnt/share/spot_maps/warehouse_v2
  - name: lab
    path: ~/spot/lab_graph

server:
  host: 127.0.0.1
  port: 8000
```

CLI flags override config values: `--config`, `--graphs`, `--host`, `--port`.

A complete example lives at `../../graphnav_viz.example.yaml`.

## Annotations sidecar

Saved as `<graph_dir>/annotations.json`:

```json
{
  "doors": [
    {"tag_id": 23, "name": "lab_door", "tag_to_handle": [0, 0, 0.12],
     "handle_type": "lever", "swing": "pull", "hinge": "right",
     "nav_waypoint": "lab-entrance", "approach_standoff_m": 1.0}
  ],
  "waypoint_pois": {
    "<wp_id>": {"name": "charging-dock", "tags": ["dock"], "notes": ""}
  },
  "rooms": [
    {"name": "kitchen", "polygon_seed": [[x, y], ...], "waypoints": ["<wp_id>", ...]}
  ],
  "no_go_zones": [
    {"name": "fragile-shelf", "polygon_seed": [[x, y], ...]}
  ]
}
```

### Fiducial categorization

Boston Dynamics reserves AprilTag id ranges for specific functions. The app
classifies every fiducial in the graph and only allows tags in the
**localization** range to become doors. Other ranges render as info-only.

| Range | Category | Door eligible? |
|---|---|---|
| 1–299 | `localization` | ✅ |
| 520–549 | `dock` | ❌ |
| 580–584 | `dock_station` | ❌ |
| 585–586 | `mission_interrupt` | ❌ |
| else | `unknown` | ❌ |

See `loader.fiducial_category` for the source of truth.

## API

### Offline (always available)

| Method | Path | Purpose |
|---|---|---|
| GET | `/api/graphs` | List graphs (name + path). |
| GET | `/api/graphs/{name}` | Waypoints / edges / fiducials / bounds in seed frame. |
| GET | `/api/graphs/{name}/all_points` | Float32 XYZ buffer of every snapshot's feature cloud, transformed into the seed frame. Cached. |
| GET | `/api/graphs/{name}/pointcloud/{snapshot_id}` | Single waypoint's feature cloud, in sensor frame. |
| GET | `/api/graphs/{name}/annotations` | Current sidecar contents. |
| PUT | `/api/graphs/{name}/annotations` | Atomic replace of sidecar (writes to `.tmp` then renames). |
| POST | `/api/graphs/{name}/export/doors` | Save annotations and export `<graph>/doors.yaml`. |

### Live mode (only with `--live`)

| Method | Path | Purpose |
|---|---|---|
| GET | `/api/live` | `{enabled, hostname, uploaded_graph}` — used by the UI to show/hide the Drive controls. |
| GET | `/api/pose` | `{x, y, z, yaw, waypoint_id, localized, uploaded_graph}` — `seed_tform_body` from `GraphNavClient.get_localization_state`. |
| GET | `/api/robot_state` | `{powered_on, estopped, estop_endpoints}`. |
| GET | `/api/nav_status` | Current `{state, destination, error?}` from the background nav worker. |
| POST | `/api/power_on` | Powers on the robot's motors. |
| POST | `/api/stand` | Blocking stand (`blocking_stand`) — brings Spot to standing height, e.g. to see fiducials mounted above sitting-camera height. |
| POST | `/api/stop` | Cancels the active nav and sends a `RobotCommandBuilder.stop_command()` (does not estop). |
| GET | `/api/estop/state` | `{level: 'allowed' \| 'settling' \| 'cut' \| 'unknown', endpoint_registered, any_other_estopped}`. |
| POST | `/api/estop/allow` | Re-arm motion (clear the software estop). |
| POST | `/api/estop/settle` | Settle-then-cut: robot sits gracefully, then motors cut. |
| POST | `/api/estop/cut` | Immediate motor power cut. |
| POST | `/api/graphs/{name}/upload` | Streams `graph` + missing waypoint/edge snapshots to the robot. |
| POST | `/api/graphs/{name}/localize` | `set_localization` with `FIDUCIAL_INIT_NEAREST`. |
| POST | `/api/graphs/{name}/navigate/{waypoint_id}` | Starts (or replaces) a background worker that drives Spot to the waypoint. |
| GET | `/api/cameras` | `[{name, image_type: 'visual' \| 'depth' \| 'unknown', cols, rows}]` from `ImageClient.list_image_sources`. |
| GET | `/api/camera/{source}` | Single JPEG snapshot from an image source. |
| GET | `/api/camera_stream/{source}` | MJPEG stream (`multipart/x-mixed-replace`) — an `<img>` tag can point straight at this. Pulls a fresh JPEG from the robot on every frame (default 8 fps, `?fps=` to override up to 15); no server-side buffering, so the stream stops as soon as the client disconnects. |

## UI tour

| 2D with scans | 3D with scans |
|---|---|
| ![2D with scans](assets/2D_graphnav_withscan.png) | ![3D with scans](assets/3D_graphnav_withscan.png) |

- **Top bar** — graph dropdown, view toggle, "Show scans", "Save annotations",
  "Export doors.yaml".
- **2D map (default)** — pan with drag, zoom with scroll. Edges as gray lines,
  waypoints as filled circles (blue if POI'd), fiducials as colored squares
  with `#id category` labels. Doors get a yellow ring.
- **"Show scans"** — fetches the union of all snapshot point clouds (in seed
  frame), colors by Z (blue floor → green mid → red ceiling), and paints them
  as an underlay in 2D and as `THREE.Points` in 3D. ~30 MB request for a
  ~180-waypoint graph; cached after the first load.
- **3D view** — three.js scene with the same primitives plus a floor grid.
  Click a waypoint to lazy-load its individual snapshot point cloud.
- **Sidebar tabs**:
  - *Waypoint*: rename / tag / notes the selected waypoint.
  - *Doors*: grouped by fiducial category. Localization tags get a "Mark as
    door" button; once marked, the full edit form appears.
  - *Rooms*: "+ New room" → click vertices on the map → double-click to
    close → name it. Waypoint membership auto-fills via point-in-polygon.
  - *No-go*: same polygon flow.
  - *Drive* (only with `--live`): power-on, upload graph to robot, localize
    via nearest fiducial, toggle "click waypoint to navigate", and a big
    red **STOP** button.
  - *Cameras* (only with `--live`): pick a body camera and view its live
    MJPEG feed. Streaming stops automatically when you leave the tab, turn
    Live off, or navigate away — it only runs while you're actually looking
    at it.

## Live mode

Pass `--live <SPOT_IP>` to start a server that's also wired to a real Spot.

What happens on startup:

1. `.env` is auto-loaded by walking up from the current directory (looks for
   `BOSDYN_CLIENT_USERNAME` / `BOSDYN_CLIENT_PASSWORD`). You don't need to
   `source .env` first.
2. The SDK authenticates and waits for time-sync.
3. The server **registers a software e-stop endpoint** (`graphnav_viz`) via
   `EstopEndpoint.force_simple_setup()`. This displaces any prior endpoint
   (e.g. the tablet's). `EstopKeepAlive` runs a daemon thread that sends a
   3 s heartbeat for the rest of the process — if the server crashes or is
   killed without cleanup, the robot will estop on its own when the
   heartbeat lapses.
4. The server **forcibly takes the body lease** (the tablet will lose body
   control) and runs a `LeaseKeepAlive` for the lifetime of the process. The
   lease and estop both return to default on Ctrl-C.

The frontend exposes a header **Live: off / on** toggle and a **Drive**
sidebar tab. While Live is on, the UI polls `/api/pose` and `/api/nav_status`
at 5 Hz and draws Spot as a cyan triangle (2D) / cone (3D), facing along its
yaw. The marker turns gray if the robot isn't localized to the loaded graph.

### Typical flow

```
1. Click "Live: on"                 (header, next to the graph dropdown)
2. Click "Power on"                 (Drive tab)
3. Click "Stand"                    (needed if fiducials are above sitting-camera height)
4. Click "Upload graph to robot"    (only the missing snapshots are streamed)
5. Click "Localize (nearest fiducial)"  → /api/pose now returns localized:true
6. Tick "Click waypoint to navigate"
7. Click any waypoint on the 2D or 3D map → robot drives there
8. STOP at any time                 (red button — also cancels the worker)
```

The header **"Live: on"** toggle and the Drive tab's **"Click waypoint to
navigate"** checkbox are two different switches — the header toggle starts
the 5 Hz pose/estop/nav-status polling that everything else in the Drive tab
depends on, and it resets to "off" on every page reload (the server-side
session state, like the uploaded graph, persists — this client-side toggle
does not). Ticking the checkbox without Live being on makes waypoint clicks
silently do nothing (no request, no dialog — they just fall back to
select/annotate behavior). The checkbox now flips Live on automatically if
it isn't already, so step 1 is a safety net rather than a hard requirement.

Do this entirely from the app — never "Acquire" from the tablet to stand the
robot manually. The server holds the lease and estop endpoint for the whole
process lifetime (see Safety notes below); the tablet taking control back
preempts both, which shows up as `LeaseUseError` spam in the server log and
`502` responses from `/api/power_on` until the tablet releases control again.

### Why a background nav worker?

`GraphNavClient.navigate_to(dst, cmd_duration=1.0)` only commands the robot
for one second. To keep Spot moving until it reaches the goal, the server
spawns a daemon thread that re-issues the same `command_id` at ~2 Hz and
polls `navigation_feedback`. The thread exits on `STATUS_REACHED_GOAL`,
any failure status (`STUCK`, `LOST`, `NO_ROUTE`, `NO_LOCALIZATION`,
`COMMAND_OVERRIDDEN`), or on `/api/stop`. Status is exposed via
`/api/nav_status` and shown in the Drive panel.

### Software e-stop

The Drive panel has three estop controls (and the same actions are mirrored
in a pop-out window via "↗ Pop out e-stop" or by opening `/estop.html`
directly):

| Button | SDK call | What it does |
|---|---|---|
| **Settle-then-cut** (orange) | `EstopKeepAlive.settle_then_cut()` | Robot sits gracefully, then motor power cuts. The recommended "soft stop". |
| **Hard cut** (red) | `EstopKeepAlive.stop()` | Immediate motor power cut — panic stop. |
| **Allow / re-arm** (green) | `EstopKeepAlive.allow()` | Clears the estop so motion is permitted again. Disabled when already armed. |

The header shows a live status badge (`● estop: armed` / `settling…` /
`CUT`). The pop-out window is a single self-contained page that talks to
the same backend, so the inline panel and the popup stay in sync.

### Safety notes

- **The server takes the lease and estop endpoint forcibly.** If someone
  else (e.g. the tablet) is driving the robot, they will be preempted.
  Don't run `--live` in shared operations without coordination.
- **The Drive "STOP" button is a stop trajectory, not an e-stop.** It
  cancels the active nav-worker and sends `RobotCommandBuilder.stop_command()`.
  Use the orange / red estop buttons for actual estop semantics.
- **This is not a hardware e-stop.** Spot's physical kill switch and
  hardware estop on the robot still govern everything; this is the
  software estop the SDK requires for any motion.
- Closing the browser tab does **not** estop the robot — the estop is
  registered server-side and stays armed until the server exits or the
  operator presses Settle / Cut. If the server process dies, the
  heartbeat lapses and Spot estops on its own.
- The robot will only respond to `/navigate` if it's powered on and not
  estopped. Both are surfaced as 409 errors with explicit messages.
- `/upload` is idempotent — re-uploading the same graph only sends snapshots
  the robot is missing.

## Integration

The exported `doors.yaml` is consumed directly by
[`spot_arm_door_fiducial`](../spot_arm_door_fiducial/), a fiducial-driven
automated door opener:

```bash
PYTHONPATH=src python -m spot_arm_door_fiducial.cli $SPOT_IP \
    --config recorded_graphs/<your_graph>/doors.yaml --door <name>
```

The integration is one-way (this app writes; the door opener reads). You can
use this app standalone — without `spot_arm_door_fiducial`, the doors tab
just records annotated YAML.

## Layout

| File | Purpose |
|---|---|
| `server.py` | FastAPI app + endpoints, CLI entry point, `RobotSession` for live mode |
| `config.py` | `AppConfig` schema + YAML loader |
| `loader.py` | Graph proto parsing, anchored layout, fiducial categorization, point-cloud transforms — no VTK |
| `schema.py` | Pydantic models for `annotations.json` |
| `export.py` | Annotations → `doors.yaml` |
| `static/` | Vanilla HTML + JS frontend (`app.js`, `map2d.js`, `map3d.js`, `style.css`, `index.html`) |

No build step on the frontend; three.js loads from a CDN via import map.

## Roadmap

- **Local-grid heightmap overlay** — each waypoint snapshot has 5 × 128×128
  grids (`terrain`, `obstacle_distance`, `no_step`, …). Decoding and
  projecting these into a stitched floor heightmap would give a sharper
  floor-plan look than the feature cloud underlay.
- **Multi-user editing** — currently single-user; no locking on
  `annotations.json`.
- **Path preview** — render the planned route before issuing `navigate_to`
  (would need `graph_nav.navigate_route`-style planning).
- **WebSocket pose stream** — replace the 5 Hz poll with push for smoother
  marker updates, useful at higher rates.

## License

This package is built on top of the Boston Dynamics Spot SDK, which is
distributed under the [Boston Dynamics Software Development Kit License
(20191101-BDSDK-SL)](../../LICENSE). Use of this package is therefore subject
to that license.
