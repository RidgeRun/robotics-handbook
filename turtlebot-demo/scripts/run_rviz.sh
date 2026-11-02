#!/usr/bin/env bash
set -eo pipefail

demo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# shellcheck source=/dev/null
source "$demo_root/env.sh"

rviz_config="${1:-$demo_root/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/rviz/tb3_gazebo.rviz}"

for topic in /scan /odom /tf; do
  if ! ros2 topic list | grep -Fxq "$topic"; then
    printf 'run_rviz.sh: warning: %s is not currently available. Start Gazebo first with ./scripts/run_gazebo.sh\n' "$topic" >&2
  fi
done

exec rviz2 -d "$rviz_config"
