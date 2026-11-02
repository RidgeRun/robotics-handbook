#!/usr/bin/env bash
set -eo pipefail

demo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
world="${1:-world}"

# shellcheck source=/dev/null
source "$demo_root/env.sh"

case "$world" in
  world)
    launch_file="turtlebot3_world.launch.py"
    ;;
  house)
    launch_file="turtlebot3_house.launch.py"
    ;;
  empty)
    launch_file="empty_world.launch.py"
    ;;
  *)
    printf 'Usage: %s [world|house|empty]\n' "$0" >&2
    exit 2
    ;;
esac

exec ros2 launch turtlebot3_gazebo "$launch_file"
