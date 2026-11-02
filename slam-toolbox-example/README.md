# Synthetic SLAM Toolbox example

This ROS 2 Jazzy demo places `slam_toolbox` between a synthetic sensor
publisher and a map subscriber:

```text
sensor_talker  -- /scan + TF -->  slam_toolbox  -- /map -->  map_listener
```

The sensor node publishes a stationary 360-degree scan of a 6 m × 6 m square
room, stationary odometry, and the required `odom → base_footprint → laser`
TF tree. The robot drives a small 0.8 m-radius circle over 30 seconds while
two large circular obstacles rotate over 37 seconds. The changing pose and
scan pattern make successive SLAM maps vary visibly. `slam_toolbox` consumes
those inputs and publishes an occupancy grid; the listener reports the map
dimensions and resolution.

Build and run from the repository root:

```bash
source ./env.sh
cd ros2-slam
colcon build --symlink-install
source install/setup.bash
ros2 launch slam_toolbox_example slam_toolbox_example.launch.py
```

Useful inspection commands in another terminal:

```bash
ros2 topic echo /map --once
ros2 topic hz /scan
ros2 run tf2_tools view_frames
```

To change how quickly the obstacles rotate:

```bash
ros2 launch slam_toolbox_example slam_toolbox_example.launch.py pattern_period:=10.0
```

The motion can also be adjusted:

```bash
ros2 launch slam_toolbox_example slam_toolbox_example.launch.py \
  motion_period:=15.0 motion_radius:=1.0
```

## Dumping maps

`map_listener` counts every `/map` message and saves every tenth map by
default. Each dump contains a PGM image and a map-server YAML file in the
current working directory's `maps/` directory:

```text
maps/map_000010.pgm
maps/map_000010.yaml
maps/map_000020.pgm
maps/map_000020.yaml
```

The corresponding inputs are saved beside each map as a JSON snapshot:

```text
maps/map_000010_inputs.json
```

Each snapshot contains the latest `/scan`, `/odom`, `/tf`, and `/tf_static`
messages available when that map update was received, including the complete
laser range array and frame metadata.

The interval and output directory can be changed at launch:

```bash
ros2 run slam_toolbox_example map_listener --ros-args \
  -p dump_every_n_maps:=5 \
  -p dump_directory:=/tmp/slam_maps
```

This is a data-flow and integration demo, not a physics simulation. The robot
does not move, so it demonstrates scan ingestion and map generation rather
than motion-based loop closure.

