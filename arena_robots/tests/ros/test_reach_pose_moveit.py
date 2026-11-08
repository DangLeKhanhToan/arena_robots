"""Tests for the MoveIt reach handler: goal building, request validation and the pure helpers of linear motion and relative targets."""

from __future__ import annotations

import math
import threading
import time
import uuid
from collections.abc import Iterator
from pathlib import Path

import pytest
import yaml

rclpy = pytest.importorskip("rclpy")

from arena_robots_msgs.action import ReachPose
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import Buffer

from arena_robots.caps import ArmSpec

_ARM_ENTRY: dict = {
    "base_link": "chassis_link",
    "tip_link": "tool0",
    "chain": ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"],
    "controller": "arm_controller",
    "moveit": {"package": "ur_moveit_config", "planning_group": "ur_manipulator"},
}


def _wait(future, timeout: float = 30.0):
    deadline = time.monotonic() + timeout
    while not future.done():
        assert time.monotonic() < deadline, "future did not complete"
        time.sleep(0.02)
    return future.result()


def _robot_dir(tmp_path: Path) -> Path:
    robot_dir = tmp_path / "robot"
    (robot_dir / "caps").mkdir(parents=True)
    (robot_dir / "model_params.yaml").write_text(yaml.dump({}))
    (robot_dir / "caps" / "arm.yaml").write_text(yaml.dump({"arm": _ARM_ENTRY}))
    return robot_dir


def _goal(**fields) -> ReachPose.Goal:
    goal = ReachPose.Goal()
    goal.target.header.frame_id = "base"
    goal.target.pose.position.x = 0.4
    goal.target.pose.orientation.w = 1.0
    for name, value in fields.items():
        setattr(goal, name, value)
    return goal


def _yaw(angle: float) -> Pose:
    pose = Pose()
    pose.orientation.z = math.sin(angle / 2)
    pose.orientation.w = math.cos(angle / 2)
    return pose


@pytest.fixture
def rig(tmp_path: Path) -> Iterator[tuple[object, ActionClient, ArmSpec]]:
    from arena_robots.bringup.arm.moveit import MoveItArmBringup
    from arena_robots.Robot import RobotView
    from arena_robots.task_server_handlers.reach_pose.moveit import ReachPoseHandlerMoveIt

    owned = not rclpy.ok()
    if owned:
        rclpy.init()
    ns = f"/reach_pose_{uuid.uuid4().hex[:8]}"
    node = rclpy.create_node("reach_pose_moveit", namespace=ns)
    bringup = MoveItArmBringup(RobotView(_robot_dir(tmp_path)), ns)
    handler = ReachPoseHandlerMoveIt(bringup, tf_buffer=Buffer(), node=node)
    group = ReentrantCallbackGroup()
    ActionServer(node, ReachPose, f"{ns}/reach_pose", execute_callback=handler.execute, cancel_callback=handler.on_cancel, callback_group=group)
    client = ActionClient(node, ReachPose, f"{ns}/reach_pose", callback_group=group)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        assert client.wait_for_server(timeout_sec=30.0)
        yield handler, client, bringup.arms()["arm"]
    finally:
        executor.shutdown()
        node.destroy_node()
        if owned:
            rclpy.shutdown()


def _run(client: ActionClient, goal: ReachPose.Goal):
    handle = _wait(client.send_goal_async(goal))
    assert handle.accepted
    return _wait(handle.get_result_async()).result


class TestBuildGoalMotion:
    def test_default_goal_uses_the_default_pipeline_at_full_speed(self, rig) -> None:
        handler, _, arm = rig
        request = handler._build_goal(_goal(), arm).request
        assert request.pipeline_id == ""
        assert request.planner_id == ""
        assert request.max_velocity_scaling_factor == 1.0
        assert request.max_acceleration_scaling_factor == 1.0

    def test_linear_goal_selects_the_pilz_lin_planner(self, rig) -> None:
        handler, _, arm = rig
        request = handler._build_goal(_goal(motion="linear"), arm).request
        assert request.pipeline_id == "pilz_industrial_motion_planner"
        assert request.planner_id == "LIN"

    def test_free_goal_keeps_the_default_pipeline(self, rig) -> None:
        handler, _, arm = rig
        request = handler._build_goal(_goal(motion="free"), arm).request
        assert request.pipeline_id == ""

    def test_speed_scales_velocity_and_acceleration(self, rig) -> None:
        handler, _, arm = rig
        request = handler._build_goal(_goal(speed=0.25), arm).request
        assert request.max_velocity_scaling_factor == pytest.approx(0.25)
        assert request.max_acceleration_scaling_factor == pytest.approx(0.25)


class TestExecuteValidation:
    def test_unknown_motion_is_aborted(self, rig) -> None:
        _, client, _ = rig
        result = _run(client, _goal(motion="spiral"))
        assert result.status == ReachPose.Result.STATUS_ABORTED
        assert "spiral" in result.reason

    def test_linear_motion_with_a_named_target_is_aborted(self, rig) -> None:
        _, client, _ = rig
        result = _run(client, _goal(motion="linear", named_target="ready"))
        assert result.status == ReachPose.Result.STATUS_ABORTED
        assert "named target" in result.reason

    def test_relative_goal_without_a_transform_names_both_frames(self, rig) -> None:
        _, client, _ = rig
        result = _run(client, _goal(relative=True))
        assert result.status == ReachPose.Result.STATUS_ABORTED
        assert "tool0" in result.reason
        assert "base" in result.reason


class TestComposeOffset:
    def test_positions_add(self) -> None:
        from arena_robots.task_server_handlers.reach_pose.moveit import _compose_offset

        tool, offset = Pose(), Pose()
        tool.position.x, tool.position.y, tool.position.z = 1.0, 2.0, 3.0
        offset.position.x, offset.position.y, offset.position.z = 0.1, -0.2, 0.3
        tool.orientation.w = offset.orientation.w = 1.0
        out = _compose_offset(tool, offset)
        assert (out.position.x, out.position.y, out.position.z) == pytest.approx((1.1, 1.8, 3.3))

    def test_offset_rotation_multiplies_the_tool_rotation(self) -> None:
        from arena_robots.task_server_handlers.reach_pose.moveit import _compose_offset

        out = _compose_offset(_yaw(math.pi / 2), _yaw(math.pi / 2))
        assert (out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w) == pytest.approx((0.0, 0.0, 1.0, 0.0), abs=1e-9)

    def test_identity_offset_keeps_the_tool_orientation(self) -> None:
        from arena_robots.task_server_handlers.reach_pose.moveit import _compose_offset

        offset = Pose()
        offset.orientation.w = 1.0
        tool = _yaw(0.7)
        out = _compose_offset(tool, offset)
        assert (out.orientation.z, out.orientation.w) == pytest.approx((tool.orientation.z, tool.orientation.w))
