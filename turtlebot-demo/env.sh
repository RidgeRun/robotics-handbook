#!/usr/bin/env bash

# Source this file from turtlebot-demo or any child directory.
_turtlebot_demo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -f /opt/ros/jazzy/setup.bash ]; then
  # shellcheck source=/dev/null
  source /opt/ros/jazzy/setup.bash
else
  printf 'env.sh: ROS 2 Jazzy was not found at /opt/ros/jazzy\n' >&2
  return 1 2>/dev/null || exit 1
fi

if [ -f "$_turtlebot_demo_root/turtlebot3_ws/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "$_turtlebot_demo_root/turtlebot3_ws/install/setup.bash"
fi

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
export TURTLEBOT3_WS="$_turtlebot_demo_root/turtlebot3_ws"

unset _turtlebot_demo_root
