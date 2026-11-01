"""Tests for arena_robots.bringup.mobile.none - NoneBringup."""

from __future__ import annotations

from unittest.mock import MagicMock


def _make_mock_robot(name: str = "test_robot") -> object:
    mock_robot = MagicMock()
    mock_robot.name = name
    return mock_robot


class TestNoneBringupProperties:
    def _make(self, namespace: str = "/robot1") -> object:
        from arena_robots.bringup.mobile.none import NoneBringup

        return NoneBringup(robot=_make_mock_robot(), namespace=namespace)

    def test_goal_topic_is_goal_pose_under_the_namespace(self):
        b = self._make("/ns1")
        assert "goal_pose" in b.goal_topic
        assert "ns1" in b.goal_topic


class TestNoneBringupLaunchActions:
    def _make(self) -> object:
        from arena_robots.bringup.mobile.none import NoneBringup

        return NoneBringup(robot=_make_mock_robot(), namespace="/robot1")

    def test_launches_one_action_and_ignores_extra_kwargs(self):
        b = self._make()
        assert len(b._launch_actions(use_sim_time=False, extra_kwarg="ignored")) == 1
