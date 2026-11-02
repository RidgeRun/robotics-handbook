#!/usr/bin/env bash
set -eo pipefail

demo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# shellcheck source=/dev/null
source "$demo_root/env.sh"

if ! ros2 pkg prefix turtlebot3_teleop >/dev/null 2>&1; then
  printf 'turtlebot3_teleop is not installed. Try:\n' >&2
  printf '  sudo apt install ros-jazzy-turtlebot3-teleop\n' >&2
  exit 1
fi

exec ros2 run turtlebot3_teleop teleop_keyboard
