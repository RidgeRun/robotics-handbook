#!/usr/bin/env bash
set -eo pipefail

demo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# shellcheck source=/dev/null
source "$demo_root/env.sh"

exec ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
