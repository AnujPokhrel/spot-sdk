"""Reusable Spot building blocks shared across tasks.

Anything that's likely to be useful for more than one workflow (auth, power,
stand, fiducial detection, SE2 walk, GraphNav, image fetch) lives here.
Task-specific logic stays in the task package.
"""

from __future__ import annotations

import math
import time
from typing import Optional

import numpy as np
from bosdyn import geometry
from bosdyn.api import basic_command_pb2, world_object_pb2
from bosdyn.api.graph_nav import graph_nav_pb2, map_pb2
from bosdyn.client import create_standard_sdk, frame_helpers
from bosdyn.client.exceptions import ResponseError
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.image import ImageClient
from bosdyn.client.math_helpers import SE2Pose, SE3Pose
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand
from bosdyn.client.util import authenticate
from bosdyn.client.world_object import WorldObjectClient


# --- Connection / lifecycle ---------------------------------------------------

def connect(hostname: str, client_name: str = 'SpotApp'):
    """Create SDK, authenticate, and wait for time sync."""
    sdk = create_standard_sdk(client_name)
    robot = sdk.create_robot(hostname)
    authenticate(robot)
    # The SDK default (3 s) times out on a jittery WiFi link.
    robot.time_sync.wait_for_sync(timeout_sec=15.0)
    return robot


def check_estop(robot) -> None:
    assert not robot.is_estopped(), (
        'Robot is estopped. Run an external E-Stop client (see python/examples/estop).')


def power_on(robot) -> None:
    robot.logger.info('Powering on robot...')
    robot.power_on(timeout_sec=20)
    assert robot.is_powered_on(), 'Robot power on failed.'


def safe_power_off(robot) -> None:
    robot.logger.info('Powering off robot...')
    robot.power_off(cut_immediately=False, timeout_sec=20)
    assert not robot.is_powered_on(), 'Robot power off failed.'


def stand(robot) -> None:
    blocking_stand(robot.ensure_client(RobotCommandClient.default_service_name), timeout_sec=10)


def pitch_up(robot, pitch_rad: float = -math.pi / 6.0, timeout_sec: float = 10.0) -> None:
    """Pitch the body so front cameras see things above eye level (e.g. door handles)."""
    cmd_client = robot.ensure_client(RobotCommandClient.default_service_name)
    footprint_R_body = geometry.EulerZXY(0.0, 0.0, pitch_rad)
    cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
    cmd_id = cmd_client.robot_command(cmd)
    end_time = time.time() + timeout_sec
    while time.time() < end_time:
        fb = cmd_client.robot_command_feedback(cmd_id)
        status = fb.feedback.synchronized_feedback.mobility_command_feedback.stand_feedback.status
        if status == basic_command_pb2.StandCommand.Feedback.STATUS_IS_STANDING:
            return
        time.sleep(0.5)
    raise RuntimeError('Failed to pitch robot up.')


# --- Images -------------------------------------------------------------------

def get_images_as_cv2(robot, sources: list[str]) -> dict:
    """Fetch image sources and decode each as a CV2 image.

    Returns {source_name: (image_proto, cv2_image)}. Requires opencv-python.
    """
    import cv2  # lazy: only tasks that fetch images need this dep.

    image_client = robot.ensure_client(ImageClient.default_service_name)
    out: dict = {}
    for response in image_client.get_image_from_sources(sources):
        buf = np.frombuffer(response.shot.image.data, dtype=np.uint8)
        out[response.source.name] = (response, cv2.imdecode(buf, -1))
    return out


# --- Fiducials ----------------------------------------------------------------

def find_fiducial(robot, tag_id: int, timeout_sec: float = 5.0,
                  poll_interval_sec: float = 0.25) -> Optional[SE3Pose]:
    """Poll WorldObjectService for a fiducial. Returns vision_T_tag or None."""
    client = robot.ensure_client(WorldObjectClient.default_service_name)
    end_time = time.time() + timeout_sec
    while time.time() < end_time:
        objs = client.list_world_objects(
            object_type=[world_object_pb2.WORLD_OBJECT_APRILTAG]).world_objects
        for obj in objs:
            if obj.apriltag_properties.tag_id != tag_id:
                continue
            pose = frame_helpers.get_a_tform_b(
                obj.transforms_snapshot, frame_helpers.VISION_FRAME_NAME,
                obj.apriltag_properties.frame_name_fiducial)
            if pose is not None:
                return pose
        time.sleep(poll_interval_sec)
    return None


def find_any_fiducial(robot, timeout_sec: float = 5.0) -> dict[int, SE3Pose]:
    """Return {tag_id: vision_T_tag} for every currently-detected fiducial."""
    client = robot.ensure_client(WorldObjectClient.default_service_name)
    end_time = time.time() + timeout_sec
    while time.time() < end_time:
        objs = client.list_world_objects(
            object_type=[world_object_pb2.WORLD_OBJECT_APRILTAG]).world_objects
        out: dict[int, SE3Pose] = {}
        for obj in objs:
            pose = frame_helpers.get_a_tform_b(
                obj.transforms_snapshot, frame_helpers.VISION_FRAME_NAME,
                obj.apriltag_properties.frame_name_fiducial)
            if pose is not None:
                out[obj.apriltag_properties.tag_id] = pose
        if out:
            return out
        time.sleep(0.25)
    return {}


# --- Movement -----------------------------------------------------------------

def walk_to_pose(robot, goal: SE2Pose, *, frame_name: str = frame_helpers.VISION_FRAME_NAME,
                 timeout_sec: float = 30.0) -> None:
    """Send a synchro SE2 trajectory and block until at goal."""
    cmd_client = robot.ensure_client(RobotCommandClient.default_service_name)
    cmd = RobotCommandBuilder.synchro_se2_trajectory_command(
        goal_se2=goal.to_proto(), frame_name=frame_name)
    end_seconds = time.time() + timeout_sec
    cmd_id = cmd_client.robot_command(cmd, end_time_secs=end_seconds)
    while time.time() < end_seconds:
        fb = cmd_client.robot_command_feedback(cmd_id)
        traj = fb.feedback.synchronized_feedback.mobility_command_feedback.se2_trajectory_feedback
        if traj.status == basic_command_pb2.SE2TrajectoryCommand.Feedback.STATUS_AT_GOAL:
            return
        time.sleep(0.25)
    raise TimeoutError('SE2 trajectory did not reach goal in time.')


def upload_graph_from_disk(robot, graph_dir) -> dict:
    """Upload a recorded GraphNav map (graph + snapshots) from a directory.

    `graph_dir` is the folder containing the `graph` proto and the
    `waypoint_snapshots/` and `edge_snapshots/` subdirs (the layout produced
    by the GraphNav recording service). Only snapshots the robot doesn't
    already have are sent. Returns counts for logging.
    """
    from pathlib import Path
    graph_dir = Path(graph_dir)
    client: GraphNavClient = robot.ensure_client(GraphNavClient.default_service_name)

    with open(graph_dir / 'graph', 'rb') as f:
        graph = map_pb2.Graph()
        graph.ParseFromString(f.read())

    generate_anchoring = not len(graph.anchoring.anchors)
    response = client.upload_graph(graph=graph, generate_new_anchoring=generate_anchoring)

    wp_uploaded = 0
    for snapshot_id in response.unknown_waypoint_snapshot_ids:
        path = graph_dir / 'waypoint_snapshots' / snapshot_id
        if not path.is_file():
            continue
        snap = map_pb2.WaypointSnapshot()
        snap.ParseFromString(path.read_bytes())
        client.upload_waypoint_snapshot(snap)
        wp_uploaded += 1

    edge_uploaded = 0
    for snapshot_id in response.unknown_edge_snapshot_ids:
        path = graph_dir / 'edge_snapshots' / snapshot_id
        if not path.is_file():
            continue
        snap = map_pb2.EdgeSnapshot()
        snap.ParseFromString(path.read_bytes())
        client.upload_edge_snapshot(snap)
        edge_uploaded += 1

    return {
        'waypoints': len(graph.waypoints),
        'edges': len(graph.edges),
        'waypoint_snapshots_uploaded': wp_uploaded,
        'edge_snapshots_uploaded': edge_uploaded,
    }


def navigate_to_waypoint(robot, waypoint: str, *, timeout_sec: float = 120.0) -> None:
    """GraphNav to a waypoint (by annotation name or full id). Blocks until arrived.

    Assumes a graph is already uploaded and the robot is localized.
    """
    client: GraphNavClient = robot.ensure_client(GraphNavClient.default_service_name)
    graph = client.download_graph()
    if graph is None:
        raise RuntimeError('No graph uploaded to the robot.')
    destination = next(
        (wp.id for wp in graph.waypoints if wp.id == waypoint or wp.annotations.name == waypoint),
        None)
    if destination is None:
        raise KeyError(f'Waypoint {waypoint!r} not found in uploaded graph.')

    nav_to_cmd_id: Optional[int] = None
    end_time = time.time() + timeout_sec
    while time.time() < end_time:
        try:
            nav_to_cmd_id = client.navigate_to(destination, 1.0, command_id=nav_to_cmd_id)
        except ResponseError as e:
            raise RuntimeError(f'navigate_to failed: {e}') from e
        time.sleep(0.5)
        s = client.navigation_feedback(nav_to_cmd_id).status
        if s == graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL:
            return
        if s in (graph_nav_pb2.NavigationFeedbackResponse.STATUS_LOST,
                 graph_nav_pb2.NavigationFeedbackResponse.STATUS_STUCK,
                 graph_nav_pb2.NavigationFeedbackResponse.STATUS_NO_ROUTE,
                 graph_nav_pb2.NavigationFeedbackResponse.STATUS_NO_LOCALIZATION,
                 graph_nav_pb2.NavigationFeedbackResponse.STATUS_COMMAND_OVERRIDDEN):
            raise RuntimeError(f'GraphNav failed with status {s}')
    raise TimeoutError(f'GraphNav to {waypoint!r} did not arrive within {timeout_sec}s')
