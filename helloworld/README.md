# Hello World ROS 2 demo

This small ROS 2 Jazzy package publishes a message on `/hello` once per second
and prints the messages received by its subscriber.

From the repository root:

```bash
source ./env.sh
cd ros2-slam
colcon build --symlink-install
source install/setup.bash
ros2 launch helloworld helloworld.launch.py
```

To run the nodes separately, use `ros2 run helloworld talker` and
`ros2 run helloworld listener` in two terminals after sourcing the workspace.
