#!/usr/bin/env bash
# Stand-in for --ros-python: runs the ROS bridge inside the ROS container while
# xarm-server stays in the host uv environment.
#
#   uv run xarm-server --backend ros --arm right --execute --ros-python ./ros_python_docker.sh
#
# The server calls this as: <ros-python> -u /host/path/ros_bridge.py --mapping {...}
# Override the container with XARM_ROS_CONTAINER.
set -euo pipefail

container=${XARM_ROS_CONTAINER:-manipulation_env_ros_$(id -un)}
[[ ${1:-} == -u ]] && shift
script=$1
shift

docker cp -q "$script" "$container:/tmp/ros_bridge.py"
exec docker exec -i -u dockeruser "$container" bash -c '
    source /opt/ros/humble/setup.bash
    source /workspace/manipulation_ws/install/setup.bash
    exec /usr/bin/python3 -u /tmp/ros_bridge.py "$@"' _ "$@"
