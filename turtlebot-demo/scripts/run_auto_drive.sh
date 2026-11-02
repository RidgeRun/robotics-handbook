#!/usr/bin/env bash
set -eo pipefail

demo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
duration_seconds="${1:-30}"
linear_x="${2:-0.16}"
angular_z="${3:-0.45}"

# shellcheck source=/dev/null
source "$demo_root/env.sh"

stop_robot() {
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true
}

trap stop_robot EXIT INT TERM

printf 'Driving TurtleBot3 for %s seconds. Press Ctrl+C to stop.\n' "$duration_seconds"
timeout "$duration_seconds" ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: $linear_x, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $angular_z}}"
