# Hello World ROS 2 C++ demo

This package contains a small ROS 2 Jazzy publisher/subscriber example written
in C++. The talker publishes a numbered `std_msgs/msg/String` message on
`/hello` once per second, and the listener logs each received message.

Build and run from the repository root:

```bash
source ./env.sh
cd ros2-slam
colcon build --symlink-install
source install/setup.bash
ros2 launch helloworld_cpp helloworld_cpp.launch.py
```

To run the nodes separately:

```bash
ros2 run helloworld_cpp talker
ros2 run helloworld_cpp listener
```
