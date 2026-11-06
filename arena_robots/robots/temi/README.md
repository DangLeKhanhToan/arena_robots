# temi

The robotemi temi (v3) telepresence robot, built from primitives since no upstream URDF exists.

Published figures it follows ([robotemi.com/specs](https://robotemi.com/specs)): approx. 1.0 m tall, 0.35 m wide, 0.45 m deep, 12 kg, 1 m/s top speed, differential drive on two hub wheels, 360 degree lidar in the base, depth and RGB cameras, IMU.

Estimated from photos, not from a spec sheet: wheel radius (0.06 m) and track (0.28 m), caster placement, the column, torso and screen geometry, the screen tilt (0.5 rad, fixed), sensor heights, and the angular limit (1.5 rad/s).

| Sensor | Component | Mount | Topic |
| --- | --- | --- | --- |
| lidar | `lidar/arena_default`, 229 degrees through the open front of the base slot | `base_slot` | `lidar` |
| depth camera | `camera/oakd_rgbd`, pitched 0.6 rad down | `torso_panel` | `depth_camera/*` |
| head camera | `camera/d435_color`, level | `screen_top` | `head_camera/*` |
| IMU | `imu/arena_default` | `imu` | `imu/data` |
