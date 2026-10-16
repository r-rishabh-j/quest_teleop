#!/usr/bin/env bash
# Teleop the real (or fake) xArm from the Quest. The robot side must be running
# in onrobot_rl_env: ./scripts/arm.sh left, then ./scripts/run_env.sh.
#
#   ./run_robot.sh                 # left arm (the one with the gripper)
#   ./run_robot.sh right           # or: both
#   ./run_robot.sh left --controller right
#
# Arguments after the arm go to xarm-vr. Ctrl+C stops the client, holds the
# arm, and closes the Quest app. Set XARM_LINK to use another endpoint.
set -euo pipefail

cd "$(dirname "$(readlink -f "$0")")"
export PATH="$HOME/.local/bin:$PATH"
endpoint=${XARM_LINK:-tcp://127.0.0.1:8765}

arm=left
case "${1:-}" in
    left|right|both) arm=$1; shift ;;
esac

./quest_prep.sh

if ! uv run xarm-client health --server "$endpoint" >/dev/null 2>&1; then
    echo "[run] waiting for run_env's Quest agent on $endpoint (onrobot_rl_env: ./scripts/run_env.sh)"
    until uv run xarm-client health --server "$endpoint" >/dev/null 2>&1; do sleep 1; done
fi
echo "[run] robot side is up; press [a] in run_env if the client says the agent is inactive"
exec uv run xarm-vr --server "$endpoint" --arm "$arm" --execute --show-input "$@"
