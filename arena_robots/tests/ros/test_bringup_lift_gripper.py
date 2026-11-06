from __future__ import annotations

import pytest

from arena_robots.bringup import BRINGUPS, AdapterCapMismatch, check_caps
from arena_robots.Robot import RobotIdentifier


@pytest.mark.parametrize("cap", ["lift", "gripper"])
def test_cap_registers_direct_and_none(cap: str):
    assert sorted(BRINGUPS[cap].keys()) == ["direct", "none"]


@pytest.mark.parametrize(("cap", "kind"), [(cap, kind) for cap in ("lift", "gripper") for kind in ("direct", "none")])
def test_bringup_requires_its_cap_and_launches_nothing(cap: str, kind: str):
    cls = BRINGUPS[cap].get(kind)
    assert cls.kind == kind
    assert cls._bringup_meta.cap == cap
    assert cls._bringup_meta.requires == frozenset({cap})
    bringup = cls(RobotIdentifier("tiago").resolve_sync(), "/env_0/tiago")
    assert bringup._launch_actions(use_sim_time=True, extra="ignored") == []
    assert bringup.accepts_task_kinds == frozenset()


@pytest.mark.parametrize("cap", ["lift", "gripper"])
def test_tiago_satisfies_the_bringup(cap: str):
    check_caps(BRINGUPS[cap].get("direct")(RobotIdentifier("tiago").resolve_sync(), "/env_0/tiago"))


@pytest.mark.parametrize("cap", ["lift", "gripper"])
def test_a_robot_without_the_cap_is_refused(cap: str):
    with pytest.raises(AdapterCapMismatch):
        check_caps(BRINGUPS[cap].get("direct")(RobotIdentifier("jackal").resolve_sync(), "/env_0/jackal"))
