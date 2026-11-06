from __future__ import annotations

from launch import Action

from arena_robots.bringup import Bringup, BringupMeta


@BringupMeta.attach(requires={"lift"}, cap="lift")
class NoneLiftBringup(Bringup):
    kind = "none"

    def _launch_actions(
        self,
        *,
        use_sim_time: bool = True,
        frame: str = "",
        **_: object,
    ) -> list[Action]:
        return []
