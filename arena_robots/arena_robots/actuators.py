"""Topic names and command clamping shared by the lift and gripper direct adapters and their MCP verbs."""

from __future__ import annotations

import math

from arena_robots.caps import GripperSpec, LiftSpec


def lift_topic(namespace: str, instance: str, leaf: str = "command") -> str:
    return f"{namespace.rstrip('/')}/lift/{instance}/{leaf}"


def gripper_topic(namespace: str, instance: str, leaf: str = "command") -> str:
    return f"{namespace.rstrip('/')}/gripper/{instance}/{leaf}"


def lift_limits(spec: LiftSpec) -> tuple[float, float] | None:
    """Joint range from the `limits: [lower, upper]` entry of the lift instance, None when undeclared."""
    raw = spec.raw.get("limits")
    if raw is None:
        return None
    if not isinstance(raw, list | tuple) or len(raw) != 2:
        raise ValueError(f"{spec.path}: lift '{spec.name}' limits must be [lower, upper]; got {raw!r}")
    lower, upper = float(raw[0]), float(raw[1])
    if not lower <= upper:
        raise ValueError(f"{spec.path}: lift '{spec.name}' limits lower {lower} exceeds upper {upper}")
    return lower, upper


def clamp_lift(spec: LiftSpec, height: float) -> float:
    if not math.isfinite(height):
        raise ValueError(f"lift height must be finite; got {height}")
    limits = lift_limits(spec)
    if limits is None:
        return height
    return min(max(height, limits[0]), limits[1])


def gripper_range(spec: GripperSpec) -> tuple[float, float]:
    positions = spec.positions
    return min(positions["open"], positions["close"]), max(positions["open"], positions["close"])


def clamp_gripper(spec: GripperSpec, position: float, max_effort: float) -> tuple[float, float]:
    """Position clamped between the open and close positions, effort capped at the spec's max_effort, 0 or less meaning the spec's max_effort."""
    if not (math.isfinite(position) and math.isfinite(max_effort)):
        raise ValueError(f"gripper position and max_effort must be finite; got {position}, {max_effort}")
    lower, upper = gripper_range(spec)
    effort = spec.max_effort if max_effort <= 0.0 else min(max_effort, spec.max_effort)
    return min(max(position, lower), upper), effort
