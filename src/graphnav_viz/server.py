"""FastAPI app: serves graphs from a parent directory + annotations + UI."""

from __future__ import annotations

import argparse
import dataclasses
import logging
import math
import threading
import time
from pathlib import Path

import numpy as np
import uvicorn
from fastapi import FastAPI, HTTPException, Response
from fastapi.responses import StreamingResponse
from fastapi.staticfiles import StaticFiles
from starlette.middleware.gzip import GZipMiddleware

from . import export, loader
from .config import AppConfig, load_config
from .schema import Annotations, Patrol


class RobotSession:
    """Optional live connection to a Spot for pose + GraphNav commands.

    Created only when the server is launched with --live <SPOT_IP>. All
    state-changing endpoints serialize on `_lock`.
    """

    def __init__(self, hostname: str):
        # Lazy imports: the offline path must work even if bosdyn-client is
        # somehow broken / not on PYTHONPATH.
        from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
        from bosdyn.client.graph_nav import GraphNavClient
        from bosdyn.client.image import ImageClient
        from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
        from bosdyn.client.robot_command import RobotCommandClient
        from bosdyn.client.robot_state import RobotStateClient
        from utils import utils

        self.hostname = hostname
        self.robot = utils.connect(hostname, client_name='graphnav_viz')
        self.graph_nav = self.robot.ensure_client(GraphNavClient.default_service_name)
        self.lease_client = self.robot.ensure_client(LeaseClient.default_service_name)
        self.command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
        self.state_client = self.robot.ensure_client(RobotStateClient.default_service_name)
        self.image_client = self.robot.ensure_client(ImageClient.default_service_name)
        # Register a software estop endpoint BEFORE taking the lease. Without
        # an active endpoint the robot reports as estopped and refuses motion.
        # force_simple_setup() displaces any existing endpoint (e.g. the
        # tablet's), matching the lease take that follows.
        self.estop_endpoint_name = 'graphnav_viz'
        self._estop_client = self.robot.ensure_client(EstopClient.default_service_name)
        self._estop_endpoint = EstopEndpoint(
            self._estop_client, name=self.estop_endpoint_name, estop_timeout=9.0)
        self._estop_endpoint.force_simple_setup()
        self.estop_keepalive = EstopKeepAlive(self._estop_endpoint)
        self.estop_keepalive.allow()  # arm motion (motor power still gated by power_on)
        # Forcibly take the body lease (the tablet usually holds it) and run
        # a keepalive in a background thread until the process exits.
        self.lease_client.take()
        self._lease_keepalive = LeaseKeepAlive(self.lease_client, return_at_exit=True)
        self.uploaded_graph: str | None = None
        self._lock = threading.Lock()
        # Background navigation thread state. navigate_to commands time out
        # after `cmd_duration` seconds, so we keep re-issuing until the goal
        # is reached, the route fails, or the user cancels.
        self._nav_thread: threading.Thread | None = None
        self._nav_cancel = threading.Event()
        self.nav_status: dict = make_status('idle')
        # Last commanded body pitch (deg, + = look up). Any nav, stand or sit
        # levels the body again, and resets this to 0.
        self.tilt_deg = 0.0

    def shutdown(self) -> None:
        """Release the estop endpoint cleanly so the tablet can resume."""
        try:
            self.estop_keepalive.shutdown()
        except Exception:  # noqa: BLE001
            pass

LOG = logging.getLogger(__name__)

STATIC_DIR = Path(__file__).parent / 'static'


class UIStaticFiles(StaticFiles):
    """StaticFiles that makes browsers revalidate on every load.

    Without a Cache-Control header, browsers cache by heuristic (~10% of the
    file's age), so after a UI update they can keep serving months-old
    index.html/app.js for days. `no-cache` still allows caching, but the
    browser checks the ETag first, which is a cheap 304 when nothing changed.
    """

    def file_response(self, *args, **kwargs):
        response = super().file_response(*args, **kwargs)
        response.headers['Cache-Control'] = 'no-cache'
        return response

# Arrival yaw tolerance for "face heading" commands. GraphNav's own default is
# unspecified and may be loose enough to swallow a whole 30° step, so heading
# goals always carry an explicit one. Tune on the robot if it hunts.
HEADING_TOLERANCE_DEG = 5.0

# Body pitch range for "look up/down". Spot clamps body orientation to what it
# can hold anyway; this keeps the UI's steps within a sane range.
MAX_TILT_DEG = 30.0


def make_status(state: str, destination: str | None = None, heading_deg: float | None = None,
                *, sweep: dict | None = None, patrol: dict | None = None, **extra) -> dict:
    """Build a nav_status dict. Every status has the same keys:

    heading_deg is the commanded map-frame heading (None = the waypoint's
    recorded orientation). sweep / patrol carry progress for those modes and
    are None otherwise. `extra` is for `error`.
    """
    return {'state': state, 'destination': destination, 'heading_deg': heading_deg,
            'sweep': sweep, 'patrol': patrol, **extra}


def quat_yaw(q) -> float:
    """Yaw (rotation about Z) of a quaternion with .w/.x/.y/.z fields."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def heading_goal_in_waypoint(wp: dict, heading_deg: float):
    """SE2Pose proto, relative to waypoint `wp`, that leaves the robot on the
    waypoint facing `heading_deg` in the map (seed) frame.

    For NavigateTo, GraphNav flattens the waypoint frame before applying
    `destination_waypoint_tform_body_goal`, so the reference is the waypoint's
    yaw-only rotation. That's the same decomposition as math_helpers'
    `Quat.to_yaw()`, and it matters for waypoints recorded on ramps/stairs.
    """
    from bosdyn.client.math_helpers import Quat, SE2Pose, recenter_angle_mod
    wp_yaw = Quat(w=wp['qw'], x=wp['qx'], y=wp['qy'], z=wp['qz']).to_yaw()
    angle = recenter_angle_mod(math.radians(heading_deg) - wp_yaw, 0.0)
    return SE2Pose(0.0, 0.0, angle).to_proto()


def discover_graphs(root: Path, max_depth: int = 2) -> dict[str, Path]:
    """Find graphs under `root` (any directory containing a `graph` proto file).

    Walks up to `max_depth` levels. Names are the path relative to `root`,
    with `/` replaced by `__` so they survive in URLs without escaping.
    If `root` itself is a graph dir, returns {root.name: root}.
    """
    if (root / 'graph').is_file():
        return {root.name: root}
    if not root.is_dir():
        return {}

    out: dict[str, Path] = {}

    def walk(d: Path, depth: int) -> None:
        if (d / 'graph').is_file():
            rel = d.relative_to(root).as_posix()
            out[rel.replace('/', '__')] = d
            return  # don't recurse below a graph dir
        if depth >= max_depth:
            return
        for child in sorted(d.iterdir()):
            if child.is_dir() and child.name not in {'waypoint_snapshots', 'edge_snapshots'}:
                walk(child, depth + 1)

    walk(root, 0)
    return out


def build_graph_map(cfg: AppConfig) -> dict[str, Path]:
    """Discover graphs under `graphs_root`, then merge in explicit entries.

    Explicit entries win on name collision. Missing/non-graph paths are
    logged as warnings and skipped, not fatal.
    """
    graphs: dict[str, Path] = {}
    if cfg.graphs_root is not None:
        graphs.update(discover_graphs(cfg.graphs_root))
    for entry in cfg.graphs:
        if not (entry.path / 'graph').is_file():
            LOG.warning('Skipping explicit graph %r: no `graph` file at %s',
                        entry.name, entry.path)
            continue
        graphs[entry.name] = entry.path
    return graphs


def create_app(cfg: AppConfig, session: RobotSession | None = None) -> FastAPI:
    graphs = build_graph_map(cfg)
    if not graphs:
        raise FileNotFoundError(
            f'No graphs found under {cfg.graphs_root} and no explicit entries.')
    cache: dict[str, loader.GraphData] = {}
    points_cache: dict[str, np.ndarray] = {}

    def _get(name: str) -> tuple[Path, loader.GraphData]:
        if name not in graphs:
            raise HTTPException(404, f'graph {name!r} not found')
        if name not in cache:
            cache[name] = loader.load(graphs[name])
        return graphs[name], cache[name]

    app = FastAPI(title='GraphNav Annotation App')
    app.add_middleware(GZipMiddleware, minimum_size=1024)

    @app.get('/api/graphs')
    def list_graphs():
        out = []
        for name, path in graphs.items():
            entry = {'name': name, 'path': str(path)}
            if name in cache:
                gd = cache[name]
                entry.update(waypoints=len(gd.waypoints), fiducials=len(gd.fiducials),
                             anchored=gd.anchored)
            out.append(entry)
        return out

    @app.get('/api/graphs/{name}')
    def get_graph(name: str):
        _, gd = _get(name)
        return dataclasses.asdict(gd)

    @app.get('/api/graphs/{name}/all_points')
    def get_all_points(name: str):
        path, _ = _get(name)
        if name not in points_cache:
            points_cache[name] = loader.load_all_points_in_seed_cached(path)
        pts = points_cache[name]
        return Response(content=pts.astype(np.float32).tobytes(),
                        media_type='application/octet-stream',
                        headers={'X-Point-Count': str(pts.shape[0])})

    @app.get('/api/graphs/{name}/pointcloud/{snapshot_id}')
    def get_pointcloud(name: str, snapshot_id: str):
        path, _ = _get(name)
        try:
            pts = loader.load_pointcloud(path, snapshot_id)
        except FileNotFoundError:
            raise HTTPException(404, f'snapshot {snapshot_id} not found')
        return Response(content=pts.astype(np.float32).tobytes(),
                        media_type='application/octet-stream')

    @app.get('/api/graphs/{name}/annotations', response_model=Annotations)
    def get_annotations(name: str):
        path, _ = _get(name)
        f = path / 'annotations.json'
        if f.exists():
            return Annotations.model_validate_json(f.read_text())
        return Annotations()

    @app.put('/api/graphs/{name}/annotations', response_model=Annotations)
    def put_annotations(name: str, ann: Annotations):
        path, _ = _get(name)
        f = path / 'annotations.json'
        tmp = f.with_suffix('.json.tmp')
        tmp.write_text(ann.model_dump_json(indent=2))
        tmp.replace(f)
        return ann

    @app.post('/api/graphs/{name}/export/doors')
    def post_export_doors(name: str):
        path, _ = _get(name)
        f = path / 'annotations.json'
        if not f.exists():
            raise HTTPException(400, 'No annotations saved yet.')
        ann = Annotations.model_validate_json(f.read_text())
        out = export.write_doors_yaml(ann, path / 'doors.yaml')
        return {'path': str(out), 'count': len(ann.doors)}

    # --- Live mode endpoints (only meaningful when --live was passed) ---

    @app.get('/api/live')
    def get_live_status():
        """Whether a live robot session is attached. Frontend uses this to
        decide whether to show the Live toggle / Drive tab."""
        return {
            'enabled': session is not None,
            'hostname': session.hostname if session else None,
            'uploaded_graph': session.uploaded_graph if session else None,
        }

    def _require_session() -> RobotSession:
        if session is None:
            raise HTTPException(503, 'Server was not started with --live; no robot connection.')
        return session

    @app.get('/api/pose')
    def get_pose():
        """Current localized pose in seed frame, plus localization status."""
        s = _require_session()
        try:
            state = s.graph_nav.get_localization_state()
        except Exception as e:  # noqa: BLE001 - surface anything from the robot to the UI
            raise HTTPException(502, f'get_localization_state failed: {e}') from e
        loc = state.localization
        seed_T_body = loc.seed_tform_body
        # If never localized, position/rotation are all zero — flag that.
        localized = bool(loc.waypoint_id) and (
            seed_T_body.rotation.w != 0 or seed_T_body.rotation.x != 0
            or seed_T_body.rotation.y != 0 or seed_T_body.rotation.z != 0)
        yaw = quat_yaw(seed_T_body.rotation)
        return {
            'localized': localized,
            'waypoint_id': loc.waypoint_id,
            'uploaded_graph': s.uploaded_graph,
            'x': seed_T_body.position.x,
            'y': seed_T_body.position.y,
            'z': seed_T_body.position.z,
            'yaw': yaw,
        }

    @app.post('/api/graphs/{name}/upload')
    def post_upload(name: str):
        s = _require_session()
        path, _ = _get(name)
        from utils import utils
        with s._lock:
            try:
                summary = utils.upload_graph_from_disk(s.robot, path)
            except Exception as e:  # noqa: BLE001
                raise HTTPException(502, f'upload failed: {e}') from e
            s.uploaded_graph = name
        return {'ok': True, 'graph': name, **summary}

    @app.post('/api/graphs/{name}/localize')
    def post_localize(name: str):
        s = _require_session()
        if s.uploaded_graph != name:
            raise HTTPException(409, f'graph {name!r} is not uploaded; upload first.')
        from bosdyn.api.graph_nav import graph_nav_pb2, nav_pb2
        with s._lock:
            try:
                resp = s.graph_nav.set_localization(
                    initial_guess_localization=nav_pb2.Localization(),
                    fiducial_init=graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_NEAREST,
                )
            except Exception as e:  # noqa: BLE001
                raise HTTPException(502, f'set_localization failed: {e}') from e
        return {'ok': True, 'waypoint_id': resp.waypoint_id}

    def _drive_until_done(s: RobotSession, cancel: threading.Event, wp: dict,
                          heading_deg: float | None) -> str:
        """Re-issue navigate_to until the robot arrives, fails, or `cancel` is set.

        With `heading_deg`, the goal is the waypoint itself rotated to that
        map-frame heading, so a robot already on the waypoint just turns in
        place. Returns 'reached', 'canceled', or a failure label; RPC errors
        propagate to the caller.
        """
        from bosdyn.api.graph_nav import graph_nav_pb2
        FAIL_STATES = {
            graph_nav_pb2.NavigationFeedbackResponse.STATUS_LOST: 'lost',
            graph_nav_pb2.NavigationFeedbackResponse.STATUS_STUCK: 'stuck',
            graph_nav_pb2.NavigationFeedbackResponse.STATUS_NO_ROUTE: 'no_route',
            graph_nav_pb2.NavigationFeedbackResponse.STATUS_NO_LOCALIZATION: 'no_localization',
            graph_nav_pb2.NavigationFeedbackResponse.STATUS_COMMAND_OVERRIDDEN: 'overridden',
        }
        body_goal = travel_params = None
        if heading_deg is not None:
            body_goal = heading_goal_in_waypoint(wp, heading_deg)
            travel_params = graph_nav_pb2.TravelParams(
                max_yaw=math.radians(HEADING_TOLERANCE_DEG))
        cmd_id = None
        while not cancel.is_set():
            cmd_id = s.graph_nav.navigate_to(
                wp['id'], 1.0, travel_params=travel_params, command_id=cmd_id,
                destination_waypoint_tform_body_goal=body_goal)
            if cancel.wait(0.5):
                break
            fb = s.graph_nav.navigation_feedback(cmd_id).status
            if fb == graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL:
                return 'reached'
            if fb in FAIL_STATES:
                return FAIL_STATES[fb]
        return 'canceled'

    def _publish(s: RobotSession, status: dict) -> None:
        # A worker that was replaced by a newer command must not overwrite
        # the newer command's status.
        if s._nav_thread is threading.current_thread():
            s.nav_status = status

    def _run_nav_worker(s: RobotSession, cancel: threading.Event, wp: dict,
                        heading_deg: float | None) -> None:
        try:
            _publish(s, make_status(_drive_until_done(s, cancel, wp, heading_deg),
                                    wp['id'], heading_deg))
        except Exception as e:  # noqa: BLE001
            LOG.exception('navigate_to failed mid-route')
            _publish(s, make_status('error', wp['id'], heading_deg,
                                    error=f'{type(e).__name__}: {e}'))

    def _sweep_status(wp: dict, headings: list[float], dwell_s: float,
                      state: str, i: int, phase: str, **extra) -> dict:
        return make_status(state, wp['id'], headings[i], **extra,
                           sweep={'index': i, 'total': len(headings), 'phase': phase,
                                  'dwell_s': dwell_s})

    def _run_sweep_worker(s: RobotSession, cancel: threading.Event, wp: dict,
                          headings: list[float], dwell_s: float) -> None:
        """Face each heading in turn at `wp`, holding still `dwell_s` at each."""
        i = 0
        def status(state: str, phase: str, **extra) -> dict:
            return _sweep_status(wp, headings, dwell_s, state, i, phase, **extra)
        try:
            for i in range(len(headings)):
                _publish(s, status('sweeping', 'turning'))
                result = _drive_until_done(s, cancel, wp, headings[i])
                if result != 'reached':
                    _publish(s, status(result, 'turning'))
                    return
                _publish(s, status('sweeping', 'dwelling'))
                if cancel.wait(dwell_s):
                    _publish(s, status('canceled', 'dwelling'))
                    return
            _publish(s, status('reached', 'done'))
        except Exception as e:  # noqa: BLE001
            LOG.exception('look-around sweep failed')
            _publish(s, status('error', 'turning', error=f'{type(e).__name__}: {e}'))

    def _patrol_status(patrol: Patrol, wps: list[dict], state: str, i: int, lap: int,
                       phase: str, **extra) -> dict:
        stop = patrol.stops[i]
        return make_status(state, wps[i]['id'], stop.heading_deg, **extra,
                           patrol={'name': patrol.name, 'index': i, 'total': len(wps),
                                   'lap': lap, 'loop': patrol.loop, 'phase': phase,
                                   'dwell_s': stop.dwell_s})

    def _run_patrol_worker(s: RobotSession, cancel: threading.Event, patrol: Patrol,
                           wps: list[dict]) -> None:
        """Visit each stop in order: drive there, face its heading, hold for its
        dwell. With `patrol.loop`, start over until canceled or a stop fails."""
        i, lap = 0, 1
        def status(state: str, phase: str, **extra) -> dict:
            return _patrol_status(patrol, wps, state, i, lap, phase, **extra)
        try:
            while True:
                for i, stop in enumerate(patrol.stops):
                    _publish(s, status('patrolling', 'driving'))
                    result = _drive_until_done(s, cancel, wps[i], stop.heading_deg)
                    if result != 'reached':
                        _publish(s, status(result, 'driving'))
                        return
                    _publish(s, status('patrolling', 'dwelling'))
                    if cancel.wait(stop.dwell_s):
                        _publish(s, status('canceled', 'dwelling'))
                        return
                if not patrol.loop:
                    break
                lap += 1
            _publish(s, status('reached', 'done'))
        except Exception as e:  # noqa: BLE001
            LOG.exception('patrol %r failed', patrol.name)
            _publish(s, status('error', 'driving', error=f'{type(e).__name__}: {e}'))

    def _start_worker(s: RobotSession, initial_status: dict, target, *args) -> None:
        """Cancel any running nav/sweep/patrol worker, then run `target(s, cancel, *args)`."""
        if s._nav_thread and s._nav_thread.is_alive():
            s._nav_cancel.set()
            s._nav_thread.join(timeout=2.0)
        # Each worker watches its own event, so one that outlives the join
        # above still sees its cancel instead of the replacement's.
        cancel = s._nav_cancel = threading.Event()

        def run():
            try:
                target(s, cancel, *args)
            finally:
                if s._nav_thread is threading.current_thread():
                    s._nav_thread = None

        s.nav_status = initial_status
        s.tilt_deg = 0.0   # GraphNav levels the body when it takes over
        t = threading.Thread(target=run, name=f'nav-{initial_status["destination"][:8]}',
                             daemon=True)
        s._nav_thread = t
        t.start()

    def _require_uploaded(s: RobotSession, name: str) -> loader.GraphData:
        if s.uploaded_graph != name:
            raise HTTPException(409, f'graph {name!r} is not uploaded; upload first.')
        return _get(name)[1]

    def _find_waypoint(gd: loader.GraphData, waypoint_id: str) -> dict:
        wp = next((w for w in gd.waypoints
                   if w['id'] == waypoint_id or w['name'] == waypoint_id), None)
        if wp is None:
            raise HTTPException(404, f'waypoint {waypoint_id!r} not in graph')
        return wp

    def _require_can_move(s: RobotSession) -> None:
        if s.robot.is_estopped():
            raise HTTPException(409, 'Robot is e-stopped. Release the e-stop first.')
        if not s.robot.is_powered_on():
            raise HTTPException(409, 'Robot motors are off. Click "Power on" in the Drive tab first.')

    def _nav_target(s: RobotSession, name: str, waypoint_id: str) -> dict:
        """Resolve a waypoint for a motion command and check the robot can move."""
        wp = _find_waypoint(_require_uploaded(s, name), waypoint_id)
        _require_can_move(s)
        return wp

    @app.post('/api/graphs/{name}/navigate/{waypoint_id}')
    def post_navigate(name: str, waypoint_id: str, heading_deg: float | None = None):
        """Drive to a waypoint. With `heading_deg` (map frame: 0 = +x, CCW
        positive), finish on the waypoint facing that heading instead of its
        recorded orientation; if already there, Spot just turns in place."""
        s = _require_session()
        if heading_deg is not None:
            if not math.isfinite(heading_deg):
                raise HTTPException(422, 'heading_deg must be a finite number.')
            heading_deg %= 360.0
        wp = _nav_target(s, name, waypoint_id)
        _start_worker(s, make_status('navigating', wp['id'], heading_deg),
                      _run_nav_worker, wp, heading_deg)
        return {'ok': True, 'destination': wp['id'], 'heading_deg': heading_deg}

    @app.post('/api/graphs/{name}/sweep/{waypoint_id}')
    def post_sweep(name: str, waypoint_id: str, step_deg: float = 30.0, dwell_s: float = 3.0):
        """Look around a full circle at a waypoint: face every `step_deg`
        heading in turn (map frame), holding `dwell_s` at each. Starts at the
        step nearest the robot's current heading and goes counter-clockwise,
        so every turn is exactly one step."""
        s = _require_session()
        count = round(360.0 / step_deg) if step_deg > 0 else 0
        if not 2 <= count <= 72 or abs(count * step_deg - 360.0) > 1e-6:
            raise HTTPException(422, 'step_deg must divide 360 evenly (5 to 180).')
        if not 0 <= dwell_s <= 120:
            raise HTTPException(422, 'dwell_s must be between 0 and 120 seconds.')
        wp = _nav_target(s, name, waypoint_id)
        try:
            seed_T_body = s.graph_nav.get_localization_state().localization.seed_tform_body
        except Exception as e:  # noqa: BLE001
            raise HTTPException(502, f'get_localization_state failed: {e}') from e
        start = round(math.degrees(quat_yaw(seed_T_body.rotation)) / step_deg) * step_deg
        headings = [(start + i * step_deg) % 360.0 for i in range(count)]
        _start_worker(s, _sweep_status(wp, headings, dwell_s, 'sweeping', 0, 'turning'),
                      _run_sweep_worker, wp, headings, dwell_s)
        return {'ok': True, 'destination': wp['id'], 'headings_deg': headings}

    @app.post('/api/graphs/{name}/patrol')
    def post_patrol(name: str, patrol: Patrol):
        """Run a patrol: visit its stops in order, facing each stop's heading
        and holding for its dwell, then stop (or start over if `loop`). A stop
        that fails (stuck, no route, ...) ends the patrol and is reported in
        nav_status. There's no re-planning around it."""
        s = _require_session()
        if not patrol.stops:
            raise HTTPException(422, 'patrol has no stops.')
        gd = _require_uploaded(s, name)
        wps = [_find_waypoint(gd, stop.waypoint_id) for stop in patrol.stops]
        _require_can_move(s)
        _start_worker(s, _patrol_status(patrol, wps, 'patrolling', 0, 1, 'driving'),
                      _run_patrol_worker, patrol, wps)
        return {'ok': True, 'stops': len(wps)}

    @app.get('/api/nav_status')
    def get_nav_status():
        s = _require_session()
        return {**s.nav_status, 'tilt_deg': s.tilt_deg}

    @app.get('/api/robot_state')
    def get_robot_state():
        s = _require_session()
        from bosdyn.api import robot_state_pb2
        try:
            rs = s.robot.ensure_client('robot-state').get_robot_state()
        except Exception as e:  # noqa: BLE001
            raise HTTPException(502, f'get_robot_state failed: {e}') from e
        # E-stop: any stop level above NONE (0) is engaged.
        estops = []
        for est in rs.estop_states:
            if est.state != robot_state_pb2.EStopState.STATE_NOT_ESTOPPED:
                estops.append(est.name)
        powered_on = (rs.power_state.motor_power_state ==
                      robot_state_pb2.PowerState.STATE_ON)
        return {
            'powered_on': powered_on,
            'estopped': bool(estops),
            'estop_endpoints': estops,
        }

    @app.post('/api/power_on')
    def post_power_on():
        s = _require_session()
        with s._lock:
            try:
                s.robot.power_on(timeout_sec=20)
            except Exception as e:  # noqa: BLE001
                LOG.exception('power_on failed')
                raise HTTPException(502, f'power_on failed: {type(e).__name__}: {e}') from e
        return {'ok': True, 'powered_on': s.robot.is_powered_on()}

    @app.post('/api/stand')
    def post_stand():
        s = _require_session()
        from bosdyn.client.robot_command import blocking_stand
        with s._lock:
            try:
                blocking_stand(s.command_client, timeout_sec=10)
            except Exception as e:  # noqa: BLE001
                LOG.exception('stand failed')
                raise HTTPException(502, f'stand failed: {type(e).__name__}: {e}') from e
        s.tilt_deg = 0.0
        return {'ok': True}

    @app.post('/api/sit')
    def post_sit():
        s = _require_session()
        from bosdyn.client.robot_command import blocking_sit
        # Stop any nav/sweep/patrol first, or its next navigate_to would stand
        # Spot right back up.
        s._nav_cancel.set()
        with s._lock:
            try:
                blocking_sit(s.command_client, timeout_sec=10)
            except Exception as e:  # noqa: BLE001
                LOG.exception('sit failed')
                raise HTTPException(502, f'sit failed: {type(e).__name__}: {e}') from e
        s.tilt_deg = 0.0
        return {'ok': True}

    @app.post('/api/tilt')
    def post_tilt(pitch_deg: float = 0.0):
        """Pitch the body to look up (+) or down (-) while standing in place.
        The feet don't move, so the heading is unchanged. The next nav, stand
        or sit levels the body again."""
        s = _require_session()
        if not math.isfinite(pitch_deg) or abs(pitch_deg) > MAX_TILT_DEG:
            raise HTTPException(422, f'pitch_deg must be within +/-{MAX_TILT_DEG:g}.')
        if s._nav_thread and s._nav_thread.is_alive():
            # GraphNav would override the stand command on its next re-issue.
            raise HTTPException(409, 'Spot is moving. Wait until it arrives, or press STOP first.')
        _require_can_move(s)
        from bosdyn import geometry
        from bosdyn.client.robot_command import RobotCommandBuilder
        # EulerZXY pitch is nose-down positive; this endpoint is look-up positive.
        cmd = RobotCommandBuilder.synchro_stand_command(
            footprint_R_body=geometry.EulerZXY(yaw=0.0, roll=0.0, pitch=-math.radians(pitch_deg)))
        with s._lock:
            try:
                s.command_client.robot_command(cmd)
            except Exception as e:  # noqa: BLE001
                LOG.exception('tilt failed')
                raise HTTPException(502, f'tilt failed: {type(e).__name__}: {e}') from e
        s.tilt_deg = pitch_deg
        return {'ok': True, 'tilt_deg': pitch_deg}

    @app.get('/api/cameras')
    def get_cameras():
        s = _require_session()
        from bosdyn.api import image_pb2
        type_names = {
            image_pb2.ImageSource.IMAGE_TYPE_VISUAL: 'visual',
            image_pb2.ImageSource.IMAGE_TYPE_DEPTH: 'depth',
        }
        try:
            sources = s.image_client.list_image_sources()
        except Exception as e:  # noqa: BLE001
            raise HTTPException(502, f'list_image_sources failed: {e}') from e
        return [
            {'name': src.name, 'image_type': type_names.get(src.image_type, 'unknown'),
             'cols': src.cols, 'rows': src.rows}
            for src in sources
        ]

    @app.get('/api/camera/{source}')
    def get_camera_frame(source: str):
        """Single JPEG snapshot from an image source. Mainly useful for testing."""
        s = _require_session()
        from bosdyn.api import image_pb2
        from bosdyn.client.image import build_image_request
        req = [build_image_request(source, image_format=image_pb2.Image.FORMAT_JPEG)]
        try:
            resp = s.image_client.get_image(req)
        except Exception as e:  # noqa: BLE001
            raise HTTPException(502, f'get_image failed: {e}') from e
        if not resp or resp[0].status != image_pb2.ImageResponse.STATUS_OK:
            raise HTTPException(502, f'image source {source!r} unavailable')
        return Response(content=resp[0].shot.image.data, media_type='image/jpeg')

    @app.get('/api/camera_stream/{source}')
    def get_camera_stream(source: str, fps: float = 8.0):
        """MJPEG stream (multipart/x-mixed-replace) — consumable directly by an <img> tag.

        Pulls a fresh JPEG from the robot's image service on every frame; there's
        no server-side buffering, so the stream stops as soon as the client
        disconnects (browser navigates away / tab closes / img.src cleared).
        """
        s = _require_session()
        from bosdyn.api import image_pb2
        from bosdyn.client.image import build_image_request
        req = [build_image_request(source, quality_percent=75,
                                    image_format=image_pb2.Image.FORMAT_JPEG)]
        period = 1.0 / max(1.0, min(fps, 15.0))

        def frames():
            while True:
                start = time.monotonic()
                try:
                    resp = s.image_client.get_image(req)
                except Exception:  # noqa: BLE001 - client disconnect or robot hiccup; just stop
                    LOG.exception('camera_stream %r: get_image failed, stopping stream', source)
                    return
                if resp and resp[0].status == image_pb2.ImageResponse.STATUS_OK:
                    jpg = resp[0].shot.image.data
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n'
                           b'Content-Length: ' + str(len(jpg)).encode() + b'\r\n\r\n'
                           + jpg + b'\r\n')
                elapsed = time.monotonic() - start
                time.sleep(max(0.0, period - elapsed))

        return StreamingResponse(frames(), media_type='multipart/x-mixed-replace; boundary=frame')

    @app.get('/api/estop/state')
    def get_estop_state():
        s = _require_session()
        from bosdyn.api import robot_state_pb2
        try:
            rs = s.state_client.get_robot_state()
        except Exception as e:  # noqa: BLE001
            raise HTTPException(502, f'get_robot_state failed: {e}') from e
        # Find our endpoint and report its level.
        level_map = {
            robot_state_pb2.EStopState.STATE_NOT_ESTOPPED: 'allowed',
            robot_state_pb2.EStopState.STATE_ESTOPPED: 'cut',
            robot_state_pb2.EStopState.STATE_UNKNOWN: 'unknown',
        }
        ours = next((e for e in rs.estop_states if e.name == s.estop_endpoint_name), None)
        any_other_estopped = any(
            e.name != s.estop_endpoint_name and
            e.state != robot_state_pb2.EStopState.STATE_NOT_ESTOPPED
            for e in rs.estop_states)
        if ours is None:
            return {'level': 'unknown', 'endpoint_registered': False,
                    'any_other_estopped': any_other_estopped}
        return {
            'level': level_map.get(ours.state, 'unknown'),
            'endpoint_registered': True,
            'any_other_estopped': any_other_estopped,
        }

    @app.post('/api/estop/allow')
    def post_estop_allow():
        s = _require_session()
        try:
            s.estop_keepalive.allow()
        except Exception as e:  # noqa: BLE001
            raise HTTPException(502, f'estop allow failed: {e}') from e
        return {'ok': True}

    @app.post('/api/estop/settle')
    def post_estop_settle():
        s = _require_session()
        s._nav_cancel.set()
        try:
            s.estop_keepalive.settle_then_cut()
        except Exception as e:  # noqa: BLE001
            raise HTTPException(502, f'estop settle_then_cut failed: {e}') from e
        return {'ok': True}

    @app.post('/api/estop/cut')
    def post_estop_cut():
        s = _require_session()
        s._nav_cancel.set()
        try:
            s.estop_keepalive.stop()
        except Exception as e:  # noqa: BLE001
            raise HTTPException(502, f'estop cut failed: {e}') from e
        return {'ok': True}

    @app.post('/api/stop')
    def post_stop():
        s = _require_session()
        from bosdyn.client.robot_command import RobotCommandBuilder
        s._nav_cancel.set()
        with s._lock:
            try:
                s.command_client.robot_command(RobotCommandBuilder.stop_command())
            except Exception as e:  # noqa: BLE001
                raise HTTPException(502, f'stop failed: {e}') from e
        s.nav_status = make_status('canceled', s.nav_status.get('destination'))
        return {'ok': True}

    app.mount('/', UIStaticFiles(directory=str(STATIC_DIR), html=True), name='static')
    return app


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=None,
                        help='Path to graphnav_viz.yaml. Defaults to '
                             './graphnav_viz.yaml if present, else built-in defaults.')
    parser.add_argument('--graphs', default=None,
                        help='Override graphs_root from the config: a parent directory '
                             'containing graph subdirectories, OR a single graph directory. '
                             'Walks up to 2 levels deep.')
    parser.add_argument('--host', default=None)
    parser.add_argument('--port', type=int, default=None)
    parser.add_argument('--live', metavar='SPOT_IP', default=None,
                        help='Connect to a Spot at this IP/hostname for live pose '
                             'overlay and click-to-navigate.')
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format='%(levelname)s %(name)s: %(message)s')

    cfg = load_config(args.config)
    if args.graphs is not None:
        cfg.graphs_root = Path(args.graphs)
    if args.host is not None:
        cfg.server.host = args.host
    if args.port is not None:
        cfg.server.port = args.port

    import atexit
    session = None
    if args.live:
        # Auto-load .env so BOSDYN_CLIENT_USERNAME/PASSWORD are available
        # without `source .env`. find_dotenv walks up from CWD.
        try:
            from dotenv import find_dotenv, load_dotenv
            env_file = find_dotenv(usecwd=True)
            if env_file:
                load_dotenv(env_file, override=True)
                import os as _os
                u = _os.environ.get('BOSDYN_CLIENT_USERNAME', '')
                p = _os.environ.get('BOSDYN_CLIENT_PASSWORD', '')
                LOG.info('Loaded %s (USERNAME=%s, PASSWORD=%s)',
                         env_file,
                         'set' if u else 'EMPTY',
                         'set' if p else 'EMPTY')
            else:
                LOG.warning('No .env found above %s — credentials must be in environment.',
                            Path.cwd())
        except ImportError:
            LOG.warning('python-dotenv not installed; .env not auto-loaded.')
        LOG.info('Connecting to Spot at %s ...', args.live)
        session = RobotSession(args.live)
        atexit.register(session.shutdown)
        LOG.info('Connected. Live mode enabled.')

    uvicorn.run(create_app(cfg, session), host=cfg.server.host, port=cfg.server.port)


if __name__ == '__main__':
    main()
