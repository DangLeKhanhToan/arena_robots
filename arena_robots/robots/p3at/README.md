# p3at

The Adept MobileRobots Pioneer 3-AT, a four-wheel skid-steer base driven by `diff_drive_controller` with two wheels per side.

Geometry and link placement follow `p2os_urdf` from [allenh1/p2os](https://github.com/allenh1/p2os) (BSD). The meshes come from that repo through the `meshes` submodule, sparse on `p2os_urdf/meshes/p3at_meshes`, fetched by `arena feature robots add p3at`. `base_link` sits at the wheel axle (0.111 m above the ground). Collisions are primitives, upstream's placeholder inertias are replaced by computed ones.

Limits follow the Pioneer 3-AT spec: 0.7 m/s linear, 2.0 rad/s angular.

| Sensor | Component | Mount | Topic |
| --- | --- | --- | --- |
| lidar | `lidar/arena_default` | `top_front` | `lidar` |
| IMU | `imu/arena_default` | `imu` | `imu/data` |
