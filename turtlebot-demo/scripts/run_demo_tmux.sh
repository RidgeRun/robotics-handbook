#!/usr/bin/env bash
set -eo pipefail

demo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
session_name="${SLAM_TMUX_SESSION:-slam_demo}"
world="${1:-world}"

if ! command -v tmux >/dev/null 2>&1; then
  printf 'tmux is not installed. Install it or run these in separate terminals:\n' >&2
  printf '  ./scripts/run_gazebo.sh %s\n' "$world" >&2
  printf '  ./scripts/run_slam.sh\n' >&2
  printf '  ./scripts/run_teleop.sh\n' >&2
  exit 1
fi

if tmux has-session -t "$session_name" 2>/dev/null; then
  printf 'tmux session "%s" already exists. Attach with:\n' "$session_name" >&2
  printf '  tmux attach -t %s\n' "$session_name" >&2
  exit 1
fi

tmux new-session -d -s "$session_name" -c "$demo_root" "./scripts/run_gazebo.sh '$world'"
tmux rename-window -t "$session_name:0" gazebo
tmux new-window -t "$session_name" -n slam -c "$demo_root" "./scripts/run_slam.sh"
tmux new-window -t "$session_name" -n teleop -c "$demo_root" "./scripts/run_teleop.sh"

printf 'Started tmux session "%s". Attach with:\n' "$session_name"
printf '  tmux attach -t %s\n' "$session_name"
