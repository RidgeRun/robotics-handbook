#!/usr/bin/env bash
set -eo pipefail

demo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# shellcheck source=/dev/null
source "$demo_root/env.sh"

printf 'Required demo topics:\n'
for topic in /clock /cmd_vel /scan /odom /tf /map; do
  if ros2 topic list | grep -Fxq "$topic"; then
    printf '  ok      %s\n' "$topic"
  else
    printf '  missing %s\n' "$topic"
  fi
done

printf '\nQuick checks:\n'
printf '  /scan means Gazebo is publishing lidar data.\n'
printf '  /odom means the robot is spawned and publishing motion state.\n'
printf '  /map appears after SLAM Toolbox is running and receiving scans.\n'
