from __future__ import annotations

from typing import ClassVar

from launch import Action
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from arena_robots.bringup import Bringup, BringupMeta
from arena_robots.task_kinds import TaskKind


def _load_goto_pose_drl() -> type:
    from arena_robots.task_server_handlers.goto_pose._passthrough import GotoPoseHandlerDrl

    return GotoPoseHandlerDrl


# caps/mobile.yaml sub-block consumed by this bringup (key: "drl"):
#
#   drl:
#     planner: str            # required, planner name, e.g. drlvo
#     observations: dict      # optional, default {}, free-form obs pipeline overrides
#       aliases: dict[str, str]
#     rate: float             # optional, default 10.0, per-tick obs rate in Hz
#     obs_policy: str         # optional, default "lossless", "lossless" | "latest_only"
#     goal_topic: str         # optional, default ~/goal_pose
#     cmd_vel_topic: str      # optional, default ~/cmd_vel
#     global_planner: str     # optional, default "nav2/navfn", "<family>/<kind>" or "none"
#     executor: str           # optional, default "bridge": the bridge runs moves / chunks itself;
#                             # "dwb": Nav2 controller_server with DWB follows them (FollowPath), lidar-only local costmap
#     people_as: str          # optional, default "object" (people are lidar obstacles); "human": + social cost layer
#                             # (personal space from arena_peds). Only with executor dwb.
#     namespace: str          # optional, default ""
#     use_sim_time: bool      # optional, default true
#
# The DrlAdapter (arena_runtime env/adapters/mobile/drl.py) owns subprocess lifecycle;
# this bringup spawns an optional global planner stack (nav2 planner_server-only by default).


EXECUTORS = ("bridge", "dwb")
PEOPLE_AS = ("object", "human")


@BringupMeta.attach(requires={"mobile"}, cap="mobile")
class DrlBringup(Bringup):
    kind = "drl"
    task_handlers: ClassVar[dict] = {TaskKind.GOTO_POSE: _load_goto_pose_drl}

    @property
    def _cfg(self) -> dict:
        return self.robot.caps.mobile.sub("drl")

    @property
    def planner(self) -> str:
        v = self._cfg.get("planner")
        if not v:
            raise ValueError(f"drl bringup for '{self.robot.name}' missing required 'planner': set caps/mobile.yaml 'drl.planner' or pass robot.mobile.planner:=<name>")
        return str(v)

    @property
    def observations(self) -> dict:
        v = self._cfg.get("observations", {})
        if not isinstance(v, dict):
            raise ValueError(f"drl bringup for '{self.robot.name}': 'drl.observations' must be a mapping")
        return v

    @property
    def rate(self) -> float:
        return float(self._cfg.get("rate", 10.0))

    @property
    def obs_policy(self) -> str:
        v = str(self._cfg.get("obs_policy", "lossless"))
        if v not in {"lossless", "latest_only"}:
            raise ValueError(f"drl bringup for '{self.robot.name}': 'drl.obs_policy' must be 'lossless' or 'latest_only'; got {v!r}")
        return v

    @property
    def goal_topic(self) -> str:
        v = self._cfg.get("goal_topic")
        return str(v) if v else self.namespace("goal_pose")

    @property
    def cmd_vel_topic(self) -> str:
        v = self._cfg.get("cmd_vel_topic")
        return str(v) if v else self.namespace("cmd_vel")

    def _launch_actions(
        self,
        *,
        use_sim_time: bool = True,
        frame: str = "",
        global_planner: str = "nav2/navfn",
        arena_env_node: str = "",
        executor: str = "bridge",
        people_as: str = "object",
        **kwargs: object,
    ) -> list[Action]:
        actions = self._global_planner_actions(use_sim_time=use_sim_time, frame=frame, global_planner=global_planner, arena_env_node=arena_env_node)
        executor, people_as = str(executor or "bridge"), str(people_as or "object")
        if executor not in EXECUTORS:
            raise ValueError(f"drl bringup for '{self.robot.name}': mobile.executor {executor!r} not in {EXECUTORS}")
        if people_as not in PEOPLE_AS:
            raise ValueError(f"drl bringup for '{self.robot.name}': mobile.people_as {people_as!r} not in {PEOPLE_AS}")
        if executor == "dwb":
            from arena_robots.bringup.mobile.nav2 import nav2_include  # noqa: PLC0415

            actions += nav2_include(
                self,
                use_sim_time=use_sim_time,
                frame=frame,
                arena_env_node=arena_env_node,
                env_namespace=str(kwargs.get("env_namespace", "")),
                sensors=kwargs.get("sensors"),  # type: ignore[arg-type]
                local_planner="dwb",
                params_overlay="vla_dwb_social" if people_as == "human" else "vla_dwb",
                controller_only=True,
            )
        return actions

    def _global_planner_actions(
        self,
        *,
        use_sim_time: bool,
        frame: str,
        global_planner: str,
        arena_env_node: str,
    ) -> list[Action]:
        from arena_planners.resolver import (  # noqa: PLC0415
            ResolverError,
            resolve_global_planner,
            split_global_planner,
        )

        try:
            parsed = split_global_planner(global_planner)
        except ValueError as exc:
            raise RuntimeError(f"drl bringup for '{self.robot.name}': invalid mobile.global_planner: {exc}") from exc

        if parsed is None:
            return []

        family, kind = parsed
        try:
            launch_path, _metadata = resolve_global_planner(family)
        except ResolverError as exc:
            raise RuntimeError(f"drl bringup for '{self.robot.name}': {exc}") from exc

        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_path)),
                launch_arguments={
                    "robot": self.robot.name,
                    "namespace": self.namespace,
                    "use_sim_time": str(use_sim_time).lower(),
                    "frame": frame,
                    "kind": kind,
                    "arena_env_node": arena_env_node,
                }.items(),
            )
        ]
