# ROS 2 SLAM examples

Small ROS 2 Jazzy demonstrations covering publishers/subscribers, RViz data,
SLAM Toolbox integration, and a scratch SLAM implementation.

See [`Installation.md`](Installation.md) for system dependencies, build
instructions, and TurtleBot3 setup.

## Requirements

- ROS 2 Jazzy with `colcon` and `ros2` available.
- `slam_toolbox` installed for the SLAM Toolbox example.

Build all examples from this repository directory:

```bash
cd ros2-slam
colcon build --symlink-install
source install/setup.bash
```

Use a new terminal and source `ros2-slam/install/setup.bash` before running
nodes.

## Examples

### `helloworld` — Python talker/listener

A minimal Python publisher/subscriber pair. The talker publishes a numbered
`std_msgs/msg/String` message on `/hello` once per second; the listener logs
each message.

```bash
ros2 launch helloworld helloworld.launch.py
```

Expected result: the terminal reports messages being published and received on
`/hello`. See [`helloworld/README.md`](helloworld/README.md).

### `helloworld-c++` — C++ talker/listener

The same publisher/subscriber idea implemented in C++ with `rclcpp`.

```bash
ros2 launch helloworld_cpp helloworld_cpp.launch.py
```

Expected result: numbered `/hello` messages are published once per second and
printed by the listener. See [`helloworld-c++/README.md`](helloworld-c++/README.md).

### `rviz` — RViz dummy data

Publishes a simple occupancy grid, laser scan, and TF tree for visualization.
RViz consumes these ROS topics and transforms.

Run the publisher:

```bash
ros2 launch rviz_dummy rviz_dummy.launch.py
```

In another terminal, open RViz:

```bash
rviz2
```

Expected result: RViz displays the dummy map and scan using the published TF
tree. See [`rviz/README.md`](rviz/README.md).

### `slam-toolbox-example` — SLAM Toolbox pipeline

A deterministic synthetic sensor feeds `/scan`, `/odom`, and TF into SLAM
Toolbox. SLAM Toolbox publishes `/map`; the map listener reports updates and
can dump PGM/YAML maps plus input snapshots.

```bash
ros2 launch slam_toolbox_example slam_toolbox_example.launch.py \
  pattern_period:=10.0 \
  publish_period:=0.1 \
  start_delay:=1.0 \
  dump_every_n_maps:=1 \
  dump_directory:=slam_maps_toolbox
```

Expected result: `map_*.pgm`, `map_*.yaml`, and `map_*_inputs.json` files are
written to `slam_maps_toolbox/`. The moving robot and rotating obstacles make
successive maps change. See
[`slam-toolbox-example/README.md`](slam-toolbox-example/README.md).

### `slam-scratch` — live SLAM from scratch

Uses the same synthetic talker and map listener through symlinks, but replaces
SLAM Toolbox with `scratch_slam`. The node performs odometry prediction, scan
matching, keyframing, loop-closure candidates, lightweight pose-graph
relaxation, and occupancy-grid publication on `/map`.

```bash
ros2 launch slam_scratch slam_scratch.launch.py \
  pattern_period:=10.0 \
  publish_period:=0.1 \
  start_delay:=1.0 \
  map_update_period:=2.0 \
  dump_every_n_maps:=1 \
  dump_directory:=slam_maps_scratch
```

Expected result: the scratch mapper consumes `/scan`, `/odom`, and TF,
publishes `/map` and an estimated `map -> odom` TF, and writes map dumps to
`slam_maps_scratch/`.

This is an educational implementation and is not expected to be pixel-
identical to SLAM Toolbox. See [`slam-scratch/README.md`](slam-scratch/README.md).

### `turtlebot-demo` — TurtleBot3 Gazebo SLAM

A more realistic simulation using TurtleBot3 in Gazebo. The workspace under
`turtlebot-demo/turtlebot3_ws` contains the TurtleBot3 simulation packages;
the helper scripts in the parent repository start Gazebo, SLAM Toolbox, RViz,
teleoperation, and optional automatic motion.

Build it with:

```bash
cd turtlebot-demo
./scripts/build_turtlebot3_ws.sh
```

Run the pipeline in separate terminals:

```bash
cd turtlebot-demo
./scripts/run_gazebo.sh world
./scripts/run_slam.sh
./scripts/run_rviz.sh
./scripts/run_teleop.sh
```

Expected result: Gazebo publishes TurtleBot3 sensor/odometry topics, SLAM
Toolbox publishes `/map`, and RViz displays the map while the robot moves.
See [`turtlebot-demo/README.md`](turtlebot-demo/README.md).

## Inspecting the graph

While any pipeline is running:

```bash
ros2 topic list
ros2 topic echo /map --once
ros2 topic hz /scan
ros2 run tf2_tools view_frames
```
