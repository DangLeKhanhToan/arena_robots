from __future__ import annotations

from pathlib import Path

import pytest
from arena_robots.actuators import clamp_gripper, clamp_lift, gripper_range, gripper_topic, lift_limits, lift_topic
from arena_robots.caps import GripperSpec, LiftSpec
from arena_robots.Robot import RobotIdentifier


def _lift(raw: dict) -> LiftSpec:
    return LiftSpec(path=Path("caps/lift.yaml"), raw={"joint": "j", "controller": "c", **raw}, name="torso")


def _gripper(raw: dict | None = None) -> GripperSpec:
    return GripperSpec(path=Path("caps/gripper.yaml"), raw={"joint": "j", "controller": "c", "positions": {"open": 0.044, "close": 0.0}, "max_effort": 16.0, **(raw or {})}, name="gripper")


class TestLift:
    def test_height_inside_the_limits_passes(self):
        assert clamp_lift(_lift({"limits": [0.0, 0.35]}), 0.2) == 0.2

    def test_height_outside_the_limits_is_clamped(self):
        spec = _lift({"limits": [0.0, 0.35]})
        assert clamp_lift(spec, 1.0) == 0.35
        assert clamp_lift(spec, -0.5) == 0.0

    def test_undeclared_limits_pass_the_height_through(self):
        spec = _lift({})
        assert lift_limits(spec) is None
        assert clamp_lift(spec, 4.0) == 4.0

    @pytest.mark.parametrize("height", [float("nan"), float("inf")])
    def test_non_finite_height_is_rejected(self, height: float):
        with pytest.raises(ValueError, match="finite"):
            clamp_lift(_lift({"limits": [0.0, 1.0]}), height)

    @pytest.mark.parametrize("limits", [[0.0], [1.0, 0.0], "x"])
    def test_malformed_limits_are_rejected(self, limits: object):
        with pytest.raises(ValueError, match="limits"):
            lift_limits(_lift({"limits": limits}))

    def test_tiago_torso_declares_its_joint_range(self):
        torso = RobotIdentifier("tiago").resolve_sync().effective_caps({}).lift["torso"]
        assert lift_limits(torso) == (0.0, 0.35)

    def test_topics_sit_under_the_robot_namespace(self):
        assert lift_topic("/env_0/tiago/", "torso") == "/env_0/tiago/lift/torso/command"
        assert lift_topic("/env_0/tiago", "torso", "applied") == "/env_0/tiago/lift/torso/applied"


class TestGripper:
    def test_range_orders_open_and_close(self):
        assert gripper_range(_gripper()) == (0.0, 0.044)
        assert gripper_range(_gripper({"positions": {"open": 0.0, "close": 0.8}})) == (0.0, 0.8)

    def test_position_is_clamped_between_open_and_close(self):
        spec = _gripper()
        assert clamp_gripper(spec, 0.02, 5.0) == (0.02, 5.0)
        assert clamp_gripper(spec, 0.5, 5.0)[0] == 0.044
        assert clamp_gripper(spec, -0.5, 5.0)[0] == 0.0

    def test_effort_defaults_to_and_is_capped_at_the_spec(self):
        spec = _gripper()
        assert clamp_gripper(spec, 0.0, 0.0)[1] == 16.0
        assert clamp_gripper(spec, 0.0, 100.0)[1] == 16.0
        assert clamp_gripper(spec, 0.0, 4.0)[1] == 4.0

    def test_non_finite_command_is_rejected(self):
        with pytest.raises(ValueError, match="finite"):
            clamp_gripper(_gripper(), float("nan"), 1.0)

    def test_topics_sit_under_the_robot_namespace(self):
        assert gripper_topic("/env_0/tiago", "gripper") == "/env_0/tiago/gripper/gripper/command"
