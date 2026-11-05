"""Franka Research 3 arm and Franka Hand components on rbkairos: rendered joint names, merged GazeboSimSystem joints, caps."""

from __future__ import annotations

import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from arena_robots.assembly import build_request, resolve
from arena_robots.caps import RobotCaps
from arena_robots.catalog import Catalog, render_control_joints, render_wrapper_xacro
from arena_robots.Robot import RobotIdentifier
from arena_simulation_setup.utils.models.urdf import _inject_ros2_control_joints

COMPONENTS_ROOT = Path(__file__).resolve().parent.parent / "components"
_XACRO = shutil.which("xacro")
_FRANKA_ARM = Path(__file__).resolve().parent.parent.parent / "deps" / "franka_description" / "robots" / "common" / "franka_arm.xacro"
_ARM_JOINTS = [f"robot_arm_fr3_joint{i}" for i in range(1, 8)]
_FINGER = "robot_gripper_fr3_finger_joint1"
_MIMIC_FINGER = "robot_gripper_fr3_finger_joint2"
_REQUEST = {"arm_mount": ["arm/fr3"], "gripper_mount": ["gripper/franka_hand"]}


def _placed() -> tuple[object, object]:
    view = RobotIdentifier("rbkairos").resolve_sync()
    assert view.assembly is not None
    request, cleared_sockets, cleared_types = build_request(view.assembly, _REQUEST)
    return view, resolve(view.assembly, request, cleared_sockets=cleared_sockets, cleared_types=cleared_types)


class TestFr3Caps:
    def test_arm_caps_resolve_chain_tip_and_poses(self) -> None:
        _, resolved = _placed()
        rc = RobotCaps(caps_dir=Path("/dev/null"), resolved=resolved, catalog=Catalog(root=COMPONENTS_ROOT), prefix="robot_")
        assert rc.arm is not None and set(rc.arm) == {"arm_mount"}
        arm = rc.arm["arm_mount"]
        assert arm.chain == _ARM_JOINTS
        assert arm.tip_link == "robot_arm_fr3_link8"
        assert arm.controller == "arm_controller"

    def test_gripper_caps_resolve_joint_and_positions(self) -> None:
        _, resolved = _placed()
        rc = RobotCaps(caps_dir=Path("/dev/null"), resolved=resolved, catalog=Catalog(root=COMPONENTS_ROOT), prefix="robot_")
        assert rc.gripper is not None and set(rc.gripper) == {"gripper_mount"}
        gripper = rc.gripper["gripper_mount"]
        assert gripper.joint == _FINGER
        assert gripper.controller == "gripper_controller"
        assert (gripper.positions, gripper.max_effort) == ({"open": 0.04, "close": 0.0}, 70.0)

    def test_wrapper_chains_hand_onto_flange(self) -> None:
        view, resolved = _placed()
        wrapper = render_wrapper_xacro(view, resolved, catalog=Catalog(root=COMPONENTS_ROOT))
        assert "<xacro:fr3_arm_mount" in wrapper
        assert 'parent="$(arg prefix)arm_fr3_link8"' in wrapper


@pytest.mark.skipif(_XACRO is None, reason="xacro CLI not on PATH; run under the Arena container (bash arena -c pytest)")
@pytest.mark.skipif(not _FRANKA_ARM.is_file(), reason="deps/franka_description not initialized")
class TestRbkairosFr3EndToEnd:
    @pytest.fixture(scope="class")
    def merged(self, tmp_path_factory: pytest.TempPathFactory) -> ET.Element:
        view, resolved = _placed()
        catalog = Catalog(root=COMPONENTS_ROOT)
        wrapper_path = tmp_path_factory.mktemp("rbkairos_fr3") / "rbkairos_fr3.urdf.xacro"
        wrapper_path.write_text(render_wrapper_xacro(view, resolved, catalog=catalog))
        rendered = ET.fromstring(subprocess.run([_XACRO, str(wrapper_path)], capture_output=True, text=True, check=True).stdout)
        _inject_ros2_control_joints(rendered, render_control_joints(resolved, catalog, prefix=view.assembly.prefix))
        return rendered

    def test_arm_and_finger_joints_render_with_prefix(self, merged: ET.Element) -> None:
        joints = {j.get("name", ""): j for j in merged.findall("joint")}
        for name in [*_ARM_JOINTS, _FINGER, _MIMIC_FINGER]:
            assert name in joints, name
        assert joints[_MIMIC_FINGER].find("mimic").get("joint") == _FINGER

    def test_flange_link_present_and_hand_hangs_on_it(self, merged: ET.Element) -> None:
        links = {el.get("name", "") for el in merged.findall("link")}
        assert {"robot_arm_fr3_link8", "robot_gripper_fr3_hand", "robot_gripper_fr3_leftfinger"} <= links

    def test_single_gazebo_control_tag_carries_arm_and_hand(self, merged: ET.Element) -> None:
        tags = merged.findall("ros2_control")
        assert len(tags) == 1
        assert [p.text for p in tags[0].iter("plugin")] == ["gz_ros2_control/GazeboSimSystem"]
        by_name = {j.get("name", ""): j for j in tags[0].iter("joint")}
        for name in [*_ARM_JOINTS, _FINGER]:
            assert [c.get("name") for c in by_name[name].findall("command_interface")] == ["position"], name
        assert by_name[_MIMIC_FINGER].find("command_interface") is None
        assert by_name[_MIMIC_FINGER].get("mimic") == "true"
