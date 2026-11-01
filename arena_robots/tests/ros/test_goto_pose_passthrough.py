"""Tests for arena_robots.task_server_handlers.goto_pose._passthrough."""

from __future__ import annotations

import pytest

from conftest import until


class TestPassthroughHandler:
    def _serve(self, rig):
        from arena_robots.task_server_handlers.goto_pose._passthrough import GotoPoseHandlerNone

        return rig.listen(rig.serve(GotoPoseHandlerNone))

    def test_publishes_the_target_on_the_goal_topic(self, goto_rig):
        received = self._serve(goto_rig)
        goto_rig.result(goto_rig.send(1.5))
        until(lambda: bool(received), "published goal")
        assert received[0].pose.position.x == pytest.approx(1.5)

    def test_succeeds_at_once_with_the_target_as_final_pose(self, goto_rig):
        from action_msgs.msg import GoalStatus

        from arena_robots_msgs.action import GotoPose

        self._serve(goto_rig)
        wrapped = goto_rig.result(goto_rig.send(1.5))
        assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
        assert wrapped.result.status == GotoPose.Result.STATUS_SUCCEEDED
        assert wrapped.result.final_pose.pose.position.x == pytest.approx(1.5)


class TestGoalWindowHandlerExternal:
    def _serve(self, rig):
        import tf2_ros

        from arena_robots.task_server_handlers.goto_pose._passthrough import GotoPoseHandlerExternal

        buffer = tf2_ros.Buffer()
        return buffer, rig.listen(rig.serve(GotoPoseHandlerExternal, tf_buffer=buffer))

    def test_publishes_the_target_on_the_goal_topic(self, goto_rig):
        buffer, received = self._serve(goto_rig)
        goto_rig.robot_at(buffer, 1.6)
        goto_rig.result(goto_rig.send(1.5, pose_tolerance=0.5))
        until(lambda: bool(received), "published goal")
        assert received[0].pose.position.x == pytest.approx(1.5)

    def test_succeeds_within_tolerance_with_the_measured_pose(self, goto_rig):
        from action_msgs.msg import GoalStatus

        from arena_robots_msgs.action import GotoPose

        buffer, _ = self._serve(goto_rig)
        goto_rig.robot_at(buffer, 0.1)
        wrapped = goto_rig.result(goto_rig.send(pose_tolerance=0.5))
        assert wrapped.status == GoalStatus.STATUS_SUCCEEDED
        assert wrapped.result.status == GotoPose.Result.STATUS_SUCCEEDED
        assert wrapped.result.final_pose.pose.position.x == pytest.approx(0.1)

    def test_stays_open_until_the_robot_is_within_tolerance(self, goto_rig):
        from arena_robots_msgs.action import GotoPose

        buffer, _ = self._serve(goto_rig)
        goto_rig.robot_at(buffer, 5.0)
        handle = goto_rig.send(pose_tolerance=0.5)
        until(lambda: bool(goto_rig.feedback), "distance feedback")
        assert goto_rig.feedback[0].feedback.distance_remaining == pytest.approx(5.0)
        until(lambda: len(goto_rig.feedback) >= 2, "a second poll while far from the goal")
        goto_rig.robot_at(buffer, 0.2)
        assert goto_rig.result(handle).result.status == GotoPose.Result.STATUS_SUCCEEDED

    def test_cancel_cancels_the_open_goal(self, goto_rig):
        from action_msgs.msg import GoalStatus

        from arena_robots_msgs.action import GotoPose

        buffer, _ = self._serve(goto_rig)
        goto_rig.robot_at(buffer, 5.0)
        handle = goto_rig.send(pose_tolerance=0.5)
        until(lambda: bool(goto_rig.feedback), "distance feedback")
        goto_rig.cancel(handle)
        wrapped = goto_rig.result(handle)
        assert wrapped.status == GoalStatus.STATUS_CANCELED
        assert wrapped.result.status == GotoPose.Result.STATUS_CANCELED

    def test_a_new_goal_supersedes_the_open_one(self, goto_rig):
        from arena_robots_msgs.action import GotoPose

        buffer, _ = self._serve(goto_rig)
        goto_rig.robot_at(buffer, 5.0)
        first = goto_rig.send(pose_tolerance=0.5)
        until(lambda: bool(goto_rig.feedback), "distance feedback")
        second = goto_rig.send(pose_tolerance=0.5)
        wrapped = goto_rig.result(first)
        assert wrapped.result.status == GotoPose.Result.STATUS_CANCELED
        assert wrapped.result.reason == "superseded by new goal"
        goto_rig.cancel(second)
        goto_rig.result(second)
