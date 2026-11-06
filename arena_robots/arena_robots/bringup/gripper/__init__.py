from arena_robots.bringup import BRINGUPS, Bringup


@BRINGUPS["gripper"].register("none")
def _load_none() -> type[Bringup]:
    from .none import NoneGripperBringup

    return NoneGripperBringup


@BRINGUPS["gripper"].register("direct")
def _load_direct() -> type[Bringup]:
    from .direct import DirectGripperBringup

    return DirectGripperBringup
