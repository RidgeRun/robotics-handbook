# TurtleBot3 SLAM demo

This example contains the TurtleBot3 simulation workspace used by the helper
scripts in the parent repository. It includes the TurtleBot3 Gazebo
simulation packages and supports running Gazebo, SLAM Toolbox, RViz, and
teleoperation together.

## Build

From inside this demo directory:

```bash
cd ros2-slam/turtlebot-demo
./scripts/build_turtlebot3_ws.sh
```

The script installs missing ROS dependencies with `rosdep` and builds the
workspace at `ros2-slam/turtlebot-demo/turtlebot3_ws`.

## Run the complete demo

Source the local environment first:

```bash
cd ros2-slam/turtlebot-demo
source ./env.sh
```

Then start the components in separate terminals:

```bash
./scripts/run_gazebo.sh world
./scripts/run_slam.sh
./scripts/run_rviz.sh
./scripts/run_teleop.sh
```

Available Gazebo worlds are `world`, `house`, and `empty`:

```bash
./scripts/run_gazebo.sh house
```

Drive the robot automatically instead of using keyboard teleoperation:

```bash
./scripts/run_auto_drive.sh 30 0.16 0.45
```

Or start Gazebo, SLAM, and teleoperation in tmux:

```bash
./scripts/run_demo_tmux.sh world
tmux attach -t slam_demo
```

## Expected result

Gazebo displays a TurtleBot3 in the selected world. The robot publishes
`/scan`, `/odom`, `/tf`, and `/clock`; SLAM Toolbox consumes the scan and
publishes `/map`; RViz displays the robot, laser scan, and growing map. Use
the topic checker to verify the pipeline:

```bash
./scripts/check_demo_topics.sh
```

The same scripts are also available at the parent repository level for
backward compatibility. The simulation requires Gazebo, TurtleBot3 packages,
SLAM Toolbox, RViz, and the appropriate ROS 2 Jazzy dependencies installed on
the system.
