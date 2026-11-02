# Installation

These examples target Ubuntu 24.04 LTS with ROS 2 Jazzy.

## 1. Install ROS 2 Jazzy

Enable Ubuntu’s `universe` repository and add the ROS 2 apt source:

```bash
sudo apt update
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
sudo apt install -y ros-jazzy-desktop
```

Source ROS 2 automatically in new terminals:

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## 2. Install build and dependency tools

```bash
sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool
```

Initialize `rosdep` once per system:

```bash
sudo rosdep init
rosdep update
```

If `rosdep init` reports that it is already initialized, just run
`rosdep update`.

## 3. Install packages used by the examples

```bash
sudo apt install -y \
  ros-jazzy-slam-toolbox \
  ros-jazzy-tf2-tools \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-turtlebot3 \
  ros-jazzy-turtlebot3-msgs \
  ros-jazzy-turtlebot3-bringup \
  ros-jazzy-turtlebot3-teleop
```

`navigation2` and `nav2-bringup` are optional for the examples in this
repository, but are useful for navigation experiments after mapping.

Set the default TurtleBot3 model:

```bash
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
export TURTLEBOT3_MODEL=burger
```

## 4. Build the lightweight examples

The TurtleBot3 simulation is a nested workspace and should be built
separately. Build the five packages at the repository level with:

```bash
cd ros2-slam
colcon build --symlink-install \
  --base-paths \
  helloworld \
  helloworld-c++ \
  rviz \
  slam-toolbox-example \
  slam-scratch
source install/setup.bash
```

Verify that the packages are available:

```bash
ros2 pkg list | grep -E 'helloworld|rviz_dummy|slam_'
```

## 5. Build the TurtleBot3 workspace

The TurtleBot3 source and its local helper scripts are under
`ros2-slam/turtlebot-demo/`:

```bash
cd ros2-slam/turtlebot-demo
./scripts/build_turtlebot3_ws.sh
source ./env.sh
```

The build script runs `rosdep` for the packages in
`turtlebot3_ws/src` and builds the nested workspace with `colcon`.

## 6. Verify the TurtleBot3 simulation

From `ros2-slam/turtlebot-demo`, start Gazebo:

```bash
source ./env.sh
./scripts/run_gazebo.sh world
```

Available worlds are `world`, `house`, and `empty`:

```bash
./scripts/run_gazebo.sh house
```

Expected result: Gazebo opens and displays a TurtleBot3 Burger. The robot
does not move until teleoperation or a velocity command is started.

## 7. Run TurtleBot3 SLAM

In another terminal:

```bash
cd ros2-slam/turtlebot-demo
source ./env.sh
./scripts/run_slam.sh
```

Start RViz in another terminal:

```bash
cd ros2-slam/turtlebot-demo
source ./env.sh
./scripts/run_rviz.sh
```

Drive the robot with keyboard teleoperation:

```bash
cd ros2-slam/turtlebot-demo
source ./env.sh
./scripts/run_teleop.sh
```

Or use the deterministic automatic driver:

```bash
./scripts/run_auto_drive.sh 30 0.16 0.45
```

Expected result: Gazebo publishes `/scan`, `/odom`, `/tf`, and `/clock`;
SLAM Toolbox publishes `/map`; and RViz displays the robot, laser scan, TF
frames, and a map that grows as the robot moves.

Check the data flow with:

```bash
./scripts/check_demo_topics.sh
```

At minimum, `/scan`, `/odom`, and `/tf` should be present before RViz can
display the robot and laser. `/map` appears after SLAM Toolbox receives scans.

For a tmux-based run of Gazebo, SLAM Toolbox, and teleoperation:

```bash
./scripts/run_demo_tmux.sh world
tmux attach -t slam_demo
```

## 8. Run the repository examples

After building `ros2-slam` and sourcing `install/setup.bash`:

```bash
ros2 launch helloworld helloworld.launch.py
ros2 launch helloworld_cpp helloworld_cpp.launch.py
ros2 launch rviz_dummy rviz_dummy.launch.py
ros2 launch slam_toolbox_example slam_toolbox_example.launch.py
ros2 launch slam_scratch slam_scratch.launch.py
```

The complete commands, parameters, and expected results are documented in
[`README.md`](README.md) and in each example directory.

## Troubleshooting

Always source the correct environment in every terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ros2-slam/install/setup.bash
source ros2-slam/turtlebot-demo/env.sh
```

If a TurtleBot3 build fails after moving the workspace, rebuild the nested
workspace with a clean CMake cache:

```bash
cd ros2-slam/turtlebot-demo/turtlebot3_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-clean-cache
```

If RViz is empty, verify that Gazebo is still running and inspect the topics:

```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /map --once
```
