#!/usr/bin/env bash
# Start the simulator and the Quest client together in one terminal.
#
#   ./run_sim.sh                      # right arm
#   ./run_sim.sh left                 # left arm; or: both
#   ./run_sim.sh right --record --task "Pick up the cube"
#   ./run_sim.sh right --demo --seconds 5      # no headset
#
# Arguments after the arm go to xarm-vr. Ctrl+C stops the client, then the server.
# Server output goes to server.log in this folder.
set -euo pipefail

cd "$(dirname "$(readlink -f "$0")")"
export PATH="$HOME/.local/bin:$PATH"

arm=right
case "${1:-}" in
    left|right|both) arm=$1; shift ;;
esac

demo=false
for arg in "$@"; do
    [[ $arg == --demo ]] && demo=true
done

command -v uv >/dev/null || { echo "uv not found; install it first" >&2; exit 1; }

if ! $demo; then
    ./quest_prep.sh  # headset checks and proximity sensor off
fi

socket=/tmp/xarm-teleop-$(id -u).sock
log=server.log

if uv run xarm-client health >/dev/null 2>&1; then
    echo "A server is already running on $socket. Stop it first." >&2
    exit 1
fi
# The server never replaces an existing socket file. No server answered, so it is stale.
rm -f "$socket"

server_pid=
cleanup() {
    trap - EXIT INT TERM
    if [[ -n $server_pid ]] && kill -0 "$server_pid" 2>/dev/null; then
        kill -TERM "$server_pid" 2>/dev/null || true
        wait "$server_pid" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

echo "[run] starting simulator (arm=$arm); log: $PWD/$log"
uv run xarm-server --backend sim --arm "$arm" --viewer >"$log" 2>&1 &
server_pid=$!

for _ in $(seq 60); do
    if ! kill -0 "$server_pid" 2>/dev/null; then
        echo "[run] simulator exited during startup:" >&2
        tail -n 20 "$log" >&2
        exit 1
    fi
    uv run xarm-client health >/dev/null 2>&1 && break
    sleep 0.5
done
if ! uv run xarm-client health >/dev/null 2>&1; then
    echo "[run] simulator did not become ready in 30 s:" >&2
    tail -n 20 "$log" >&2
    exit 1
fi

echo "[run] simulator ready; starting Quest client"
status=0
if $demo; then
    uv run xarm-vr --arm "$arm" "$@" || status=$?
else
    uv run xarm-vr --arm "$arm" --controller auto --show-input "$@" || status=$?
fi

echo "[run] client stopped; stopping simulator"
exit "$status"
