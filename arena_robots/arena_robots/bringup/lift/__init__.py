from arena_robots.bringup import BRINGUPS, Bringup


@BRINGUPS["lift"].register("none")
def _load_none() -> type[Bringup]:
    from .none import NoneLiftBringup

    return NoneLiftBringup


@BRINGUPS["lift"].register("direct")
def _load_direct() -> type[Bringup]:
    from .direct import DirectLiftBringup

    return DirectLiftBringup
