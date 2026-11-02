# RViz dummy talker

This small ROS 2 package publishes dummy data for the existing
[`rviz/slam_demo.rviz`](../../rviz/slam_demo.rviz) configuration:

- `/map`: a 6 m square occupancy grid
- `/scan`: a circular laser scan
- `/tf` and `/tf_static`: the frames required to display the scan in `map`

Build and run the talker:

```bash
source ./env.sh
cd ros2-slam
colcon build --symlink-install
source install/setup.bash
ros2 launch rviz_dummy rviz_dummy.launch.py
```

In another terminal, open RViz with the repository configuration:

```bash
./scripts/run_rviz.sh
```
