# tiago

The PAL Robotics TIAGo with a PMB2 differential-drive base, torso lift, 7-DOF arm and the PAL parallel gripper.

The description is the upstream `tiago.urdf.xacro` from [pal-robotics/tiago_robot](https://github.com/pal-robotics/tiago_robot) (Apache-2.0) rendered once with `base_type:=pmb2 end_effector:=pal-gripper ft_sensor:=no-ft-sensor wrist_model:=wrist-2017` and its sensors, sonars, microphones, antennas and dock removed. `base_footprint` is dropped, so `base_link` is the root at the wheel axle (0.0985 m above the ground). The head joints are fixed at zero, the left finger mimics the right.

Departures from upstream: the wheel effort limit is 50 Nm instead of 6, the casters sit 3 mm above the ground, the torso and head collide through their STL visual meshes instead of the Y_UP-tagged DAE collision meshes, and `arm_tool_link` carries the inertia of its collision cylinder instead of a zero tensor.

Meshes come from three submodules under `meshes/`, fetched by `arena feature robots add tiago`:

| Path | Upstream | Sparse |
| --- | --- | --- |
| `meshes/tiago` | [pal-robotics/tiago_robot](https://github.com/pal-robotics/tiago_robot) | `tiago_description/meshes` |
| `meshes/pmb2` | [pal-robotics/pmb2_robot](https://github.com/pal-robotics/pmb2_robot) | `pmb2_description/meshes` |
| `meshes/pal_gripper` | [pal-robotics/pal_gripper](https://github.com/pal-robotics/pal_gripper) | `pal_gripper_description/meshes` |

| Cap | Instance | Joints | Controller |
| --- | --- | --- | --- |
| arm | `arm` | `arm_1_joint` to `arm_7_joint`, spawned in `stow` | `arm_controller` |
| lift | `torso` | `torso_lift_joint`, 0 to 0.35 m | `torso_controller` |
| gripper | `gripper` | `gripper_right_finger_joint` | `gripper_controller` |

| Sensor | Component | Mount | Topic |
| --- | --- | --- | --- |
| lidar | `lidar/arena_default`, 200 degrees | `base_front` | `lidar` |
| head camera | `camera/oakd_rgbd` | `head` | `head_camera/*` |
| IMU | `imu/arena_default` | `imu` | `imu/data` |
