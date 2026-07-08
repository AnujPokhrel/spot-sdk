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
from .schema import Annotations


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
        self.nav_status: dict = {'state': 'idle', 'destination': None}

    def shutdown(self) -> None:
        """Release the estop endpoint cleanly so the tablet can resume."""
        try:
            self.estop_keepalive.shutdown()
        except Exception:  # noqa: BLE001
            pass

LOG = logging.getLogger(__name__)

STATIC_DIR = Path(__file__).parent / 'static'


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
        # Yaw from quaternion (rotation about Z).
        q = seed_T_body.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
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

    def _run_nav_worker(s: RobotSession, destination_id: str) -> None:
        """Re-issue navigate_to until the robot arrives, fails, or is canceled."""
        from bosdyn.api.graph_nav import graph_nav_pb2
        cmd_id = None
        FAIL_STATES = {
            graph_nav_pb2.NavigationFeedbackResponse.STATUS_LOST: 'lost',
            graph_nav_pb2.NavigationFeedbackResponse.STATUS_STUCK: 'stuck',
            graph_nav_pb2.NavigationFeedbackResponse.STATUS_NO_ROUTE: 'no_route',
            graph_nav_pb2.NavigationFeedbackResponse.STATUS_NO_LOCALIZATION: 'no_localization',
            graph_nav_pb2.NavigationFeedbackResponse.STATUS_COMMAND_OVERRIDDEN: 'overridden',
        }
        try:
            while not s._nav_cancel.is_set():
                try:
                    cmd_id = s.graph_nav.navigate_to(destination_id, 1.0, command_id=cmd_id)
                except Exception as e:  # noqa: BLE001
                    LOG.exception('navigate_to failed mid-route')
                    s.nav_status = {'state': 'error', 'destination': destination_id,
                                    'error': f'{type(e).__name__}: {e}'}
                    return
                if s._nav_cancel.wait(0.5):
                    break
                fb = s.graph_nav.navigation_feedback(cmd_id).status
                if fb == graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL:
                    s.nav_status = {'state': 'reached', 'destination': destination_id}
                    return
                if fb in FAIL_STATES:
                    s.nav_status = {'state': FAIL_STATES[fb], 'destination': destination_id}
                    return
            s.nav_status = {'state': 'canceled', 'destination': destination_id}
        finally:
            s._nav_thread = None

    @app.post('/api/graphs/{name}/navigate/{waypoint_id}')
    def post_navigate(name: str, waypoint_id: str):
        s = _require_session()
        if s.uploaded_graph != name:
            raise HTTPException(409, f'graph {name!r} is not uploaded; upload first.')
        _, gd = _get(name)
        wp = next((w for w in gd.waypoints
                   if w['id'] == waypoint_id or w['name'] == waypoint_id), None)
        if wp is None:
            raise HTTPException(404, f'waypoint {waypoint_id!r} not in graph')
        if s.robot.is_estopped():
            raise HTTPException(409, 'Robot is e-stopped. Release the e-stop before navigating.')
        if not s.robot.is_powered_on():
            raise HTTPException(409, 'Robot motors are off. Click "Power on" in the Drive tab first.')

        # Cancel any existing navigation before starting a new one.
        if s._nav_thread and s._nav_thread.is_alive():
            s._nav_cancel.set()
            s._nav_thread.join(timeout=2.0)
        s._nav_cancel = threading.Event()
        s.nav_status = {'state': 'navigating', 'destination': wp['id']}
        t = threading.Thread(target=_run_nav_worker, args=(s, wp['id']),
                             name=f'nav-{wp["id"][:8]}', daemon=True)
        s._nav_thread = t
        t.start()
        return {'ok': True, 'destination': wp['id']}

    @app.get('/api/nav_status')
    def get_nav_status():
        s = _require_session()
        return s.nav_status

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
        return {'ok': True}

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
        s.nav_status = {'state': 'canceled', 'destination': s.nav_status.get('destination')}
        return {'ok': True}

    app.mount('/', StaticFiles(directory=str(STATIC_DIR), html=True), name='static')
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
