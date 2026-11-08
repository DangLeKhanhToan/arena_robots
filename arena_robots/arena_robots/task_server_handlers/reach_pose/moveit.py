from __future__ import annotations

import copy
from typing import TYPE_CHECKING

import rclpy.time
from action_msgs.msg import GoalStatus
from arena_robots_msgs.action import ReachPose
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive
from tf2_ros import TransformException

from arena_robots.lockstep_beat import LockstepBeat
from arena_robots.task_server_handlers import NativeGoals, TaskHandler, _executor_sleep

if TYPE_CHECKING:
    from rclpy.action import CancelResponse
    from rclpy.action.server import ServerGoalHandle
    from rclpy.node import Node
    from tf2_ros import Buffer

    from arena_robots.bringup.arm.moveit import MoveItArmBringup
    from arena_robots.caps import ArmSpec

_DEFAULT_POSITION_TOLERANCE = 0.01
_DEFAULT_ORIENTATION_TOLERANCE = 0.1
_DEFAULT_PLANNING_TIME = 5.0
_ARM_BEAT_PERIOD = 0.1
_PILZ = "pilz_industrial_motion_planner"
_MOTIONS = ("", "free", "linear")


def _quat_mul(a: Pose, b: Pose) -> tuple[float, float, float, float]:
    qa, qb = a.orientation, b.orientation
    return (
        qa.w * qb.x + qa.x * qb.w + qa.y * qb.z - qa.z * qb.y,
        qa.w * qb.y - qa.x * qb.z + qa.y * qb.w + qa.z * qb.x,
        qa.w * qb.z + qa.x * qb.y - qa.y * qb.x + qa.z * qb.w,
        qa.w * qb.w - qa.x * qb.x - qa.y * qb.y - qa.z * qb.z,
    )


def _compose_offset(tool: Pose, offset: Pose) -> Pose:
    """The pose `offset` applied to `tool`: positions add, the offset rotation multiplies on the left."""
    out = Pose()
    out.position.x = tool.position.x + offset.position.x
    out.position.y = tool.position.y + offset.position.y
    out.position.z = tool.position.z + offset.position.z
    out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = _quat_mul(offset, tool)
    return out


def _translate_moveit_status(code: int) -> tuple[int, str]:
    if code == MoveItErrorCodes.SUCCESS:
        return ReachPose.Result.STATUS_SUCCEEDED, ""
    if code == MoveItErrorCodes.NO_IK_SOLUTION:
        return ReachPose.Result.STATUS_NO_IK, "MoveIt: no IK solution"
    if code == MoveItErrorCodes.PLANNING_FAILED:
        return ReachPose.Result.STATUS_NO_PLAN, "MoveIt: planning failed"
    if code == MoveItErrorCodes.INVALID_MOTION_PLAN:
        return ReachPose.Result.STATUS_NO_PLAN, "MoveIt: motion plan invalid"
    if code == MoveItErrorCodes.TIMED_OUT:
        return ReachPose.Result.STATUS_TIMEOUT, "MoveIt: timed out"
    return ReachPose.Result.STATUS_ABORTED, f"MoveIt: error code {code}"


class ReachPoseHandlerMoveIt(TaskHandler[ReachPose.Goal, ReachPose.Feedback, ReachPose.Result]):
    def __init__(self, bringup: MoveItArmBringup, *, tf_buffer: Buffer, node: Node) -> None:
        self._bringup = bringup
        self._tf_buffer = tf_buffer
        self._node = node
        self._tf_prefix = bringup.frame
        self._native_clients: dict[str, ActionClient] = {mount: ActionClient(node, MoveGroup, str(bringup.arm_namespace(mount)("move_action"))) for mount in bringup.arms()}
        self._beat = LockstepBeat(node, "reach")
        self._native_goals = NativeGoals()
        for mount, arm in bringup.arms().items():
            node.create_subscription(
                JointTrajectoryControllerState,
                str(bringup.arm_namespace(mount)(arm.controller, "controller_state")),
                self._on_controller_state,
                10,
            )

    def _on_controller_state(self, msg: JointTrajectoryControllerState) -> None:
        self._beat.pulse(msg.header.stamp)

    def _tool_pose(self, arm: ArmSpec, frame: str) -> Pose | None:
        try:
            transform = self._tf_buffer.lookup_transform(frame, self._tf_prefix + arm.tip_link, rclpy.time.Time())
        except TransformException:
            return None
        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        return pose

    def on_cancel(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        return self._native_goals.cancel(goal_handle)

    async def execute(self, goal_handle: ServerGoalHandle) -> ReachPose.Result:
        arena_goal: ReachPose.Goal = goal_handle.request
        result = ReachPose.Result()

        arms = self._bringup.arms()
        if arena_goal.instance:
            if arena_goal.instance not in arms:
                result.status = ReachPose.Result.STATUS_ABORTED
                result.reason = f"unknown arm instance {arena_goal.instance!r}; available: {sorted(arms)}"
                return result
            mount = arena_goal.instance
        else:
            if len(arms) != 1:
                result.status = ReachPose.Result.STATUS_ABORTED
                result.reason = f"'instance' required; multiple arms available: {sorted(arms)}"
                return result
            (mount,) = arms
        arm = arms[mount]
        native_client = self._native_clients[mount]

        if arena_goal.motion not in _MOTIONS or (arena_goal.motion == "linear" and arena_goal.named_target):
            goal_handle.abort()
            result.status = ReachPose.Result.STATUS_ABORTED
            result.reason = f"motion {arena_goal.motion!r} unsupported{' with a named target' if arena_goal.named_target else ''}, known: {list(_MOTIONS)}"
            return result

        if arena_goal.relative and not arena_goal.named_target:
            frame = arena_goal.target.header.frame_id
            tool = self._tool_pose(arm, frame)
            if tool is None:
                goal_handle.abort()
                result.status = ReachPose.Result.STATUS_ABORTED
                result.reason = f"relative reach: no transform from {self._tf_prefix + arm.tip_link!r} to {frame!r}"
                return result
            arena_goal = copy.deepcopy(arena_goal)
            arena_goal.target.pose = _compose_offset(tool, arena_goal.target.pose)

        mg_goal = self._build_goal(arena_goal, arm)
        if mg_goal is None:
            goal_handle.abort()
            result.status = ReachPose.Result.STATUS_UNKNOWN_NAMED
            result.reason = f"named pose {arena_goal.named_target!r} not in caps/arm.yaml.named_poses"
            return result

        while not native_client.server_is_ready():
            if not goal_handle.is_active:
                result.status = ReachPose.Result.STATUS_CANCELED
                result.reason = "goal canceled by client"
                return result
            await _executor_sleep(self._node, 0.1, wall=True)

        await self._beat.acquire(_ARM_BEAT_PERIOD)
        try:
            frame = self._tf_prefix + arm.base_link if arena_goal.named_target else arena_goal.target.header.frame_id
            return await self._dispatch(goal_handle, native_client, mg_goal, result, arm, frame)
        finally:
            await self._beat.release()

    @staticmethod
    def _publish_feedback(goal_handle: ServerGoalHandle, state: str, progress: float) -> None:
        feedback = ReachPose.Feedback()
        feedback.state = state
        feedback.progress = progress
        goal_handle.publish_feedback(feedback)

    async def _dispatch(self, goal_handle: ServerGoalHandle, native_client: ActionClient, mg_goal: MoveGroup.Goal, result: ReachPose.Result, arm: ArmSpec, frame: str) -> ReachPose.Result:
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.status = ReachPose.Result.STATUS_CANCELED
            result.reason = "goal canceled by client"
            return result

        self._publish_feedback(goal_handle, "PLANNING", 0.0)
        send_future = native_client.send_goal_async(mg_goal)
        native_handle = await send_future

        if native_handle is None or not native_handle.accepted:
            goal_handle.abort()
            result.status = ReachPose.Result.STATUS_ABORTED
            result.reason = "MoveIt MotionPlanRequest aborted"
            return result

        self._publish_feedback(goal_handle, "EXECUTING", 0.5)
        with self._native_goals.tracking(goal_handle, native_handle):
            wrapped = await native_handle.get_result_async()
        if not goal_handle.is_active:
            result.status = ReachPose.Result.STATUS_CANCELED
            result.reason = "goal canceled by client"
            return result

        if wrapped.result is not None:
            arena_status, arena_reason = _translate_moveit_status(wrapped.result.error_code.val)
        else:
            arena_status, arena_reason = ReachPose.Result.STATUS_ABORTED, "MoveIt MotionPlanRequest aborted"
        if wrapped.status == GoalStatus.STATUS_CANCELED:
            arena_status = ReachPose.Result.STATUS_CANCELED
            arena_reason = "goal canceled by client"

        if arena_status == ReachPose.Result.STATUS_SUCCEEDED:
            self._publish_feedback(goal_handle, "DONE", 1.0)
            goal_handle.succeed()
        elif arena_status == ReachPose.Result.STATUS_CANCELED:
            goal_handle.canceled()
        else:
            goal_handle.abort()

        result.status = arena_status
        result.reason = arena_reason
        self._fill_final_pose(result, arm, frame)
        return result

    def _fill_final_pose(self, result: ReachPose.Result, arm: ArmSpec, frame: str) -> None:
        pose = self._tool_pose(arm, frame)
        if pose is not None:
            result.final_pose = PoseStamped()
            result.final_pose.header.frame_id = frame
            result.final_pose.pose = pose

    def _build_goal(self, arena_goal: ReachPose.Goal, arm: ArmSpec) -> MoveGroup.Goal | None:
        constraints = Constraints()

        if arena_goal.named_target:
            joints = arm.named_poses.get(arena_goal.named_target)
            if joints is None:
                return None
            for joint_name, position in joints.items():
                jc = JointConstraint()
                jc.joint_name = joint_name
                jc.position = float(position)
                jc.tolerance_above = 0.01
                jc.tolerance_below = 0.01
                jc.weight = 1.0
                constraints.joint_constraints.append(jc)  # pyright: ignore[reportAttributeAccessIssue]
        else:
            pos_tol = arena_goal.position_tolerance or _DEFAULT_POSITION_TOLERANCE
            ori_tol = arena_goal.orientation_tolerance or _DEFAULT_ORIENTATION_TOLERANCE

            pc = PositionConstraint()
            pc.header = arena_goal.target.header
            pc.link_name = self._tf_prefix + arm.tip_link
            prim = SolidPrimitive()
            prim.type = SolidPrimitive.SPHERE
            prim.dimensions = [max(float(pos_tol), 0.001)]
            pc.constraint_region.primitives.append(prim)  # pyright: ignore[reportAttributeAccessIssue]
            pc.constraint_region.primitive_poses.append(arena_goal.target.pose)  # pyright: ignore[reportAttributeAccessIssue]
            pc.weight = 1.0
            constraints.position_constraints.append(pc)  # pyright: ignore[reportAttributeAccessIssue]

            oc = OrientationConstraint()
            oc.header = arena_goal.target.header
            oc.link_name = self._tf_prefix + arm.tip_link
            oc.orientation = arena_goal.target.pose.orientation
            oc.absolute_x_axis_tolerance = float(ori_tol)
            oc.absolute_y_axis_tolerance = float(ori_tol)
            oc.absolute_z_axis_tolerance = float(ori_tol)
            oc.weight = 1.0
            constraints.orientation_constraints.append(oc)  # pyright: ignore[reportAttributeAccessIssue]

        mg_goal = MoveGroup.Goal()
        mg_goal.request = MotionPlanRequest()
        mg_goal.request.group_name = arm.planning_group or arm.name
        mg_goal.request.goal_constraints.append(constraints)  # pyright: ignore[reportAttributeAccessIssue]
        mg_goal.request.allowed_planning_time = float(arena_goal.planning_time or _DEFAULT_PLANNING_TIME)
        mg_goal.request.num_planning_attempts = 5
        speed = float(arena_goal.speed or 1.0)
        mg_goal.request.max_velocity_scaling_factor = speed
        mg_goal.request.max_acceleration_scaling_factor = speed
        if arena_goal.motion == "linear":
            mg_goal.request.pipeline_id = _PILZ
            mg_goal.request.planner_id = "LIN"
        # is_diff=true means "use the current robot state as start; ignore the (empty) joint_state I'm sending".
        # Without this, MoveIt warns every goal that it's ignoring our supplied start state.
        mg_goal.request.start_state.is_diff = True
        ws = arm.workspace
        if ws is not None and ws.get("type") == "box":
            ws_min, ws_max = ws["min"], ws["max"]
            if not isinstance(ws_min, list) or not isinstance(ws_max, list):
                raise ValueError(f"{arm.path}: arm '{arm.name}' workspace 'min' and 'max' must be lists")
            mg_goal.request.workspace_parameters.header.frame_id = self._tf_prefix + str(ws.get("frame", "base_link"))
            mg_goal.request.workspace_parameters.min_corner.x = float(ws_min[0])
            mg_goal.request.workspace_parameters.min_corner.y = float(ws_min[1])
            mg_goal.request.workspace_parameters.min_corner.z = float(ws_min[2])
            mg_goal.request.workspace_parameters.max_corner.x = float(ws_max[0])
            mg_goal.request.workspace_parameters.max_corner.y = float(ws_max[1])
            mg_goal.request.workspace_parameters.max_corner.z = float(ws_max[2])
        mg_goal.planning_options.plan_only = False
        return mg_goal
