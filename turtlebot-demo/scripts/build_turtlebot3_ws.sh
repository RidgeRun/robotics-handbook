#!/usr/bin/env bash
set -eo pipefail

demo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# shellcheck source=/dev/null
source "$demo_root/env.sh"

cd "$TURTLEBOT3_WS"
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-clean-cache
