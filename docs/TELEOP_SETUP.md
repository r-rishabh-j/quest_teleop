# xArm Quest teleop on roam03: setup and run guide

How to drive the xArm from a Quest 3 on this machine, with ROS running in a
container and everything else running on the host under uv. Paths are relative
to `~/experiments`.

## 1. How the pieces fit

```text
Quest 3 ──USB/adb──► xarm-vr (host, uv)
                        │ Unix socket
                        ▼
                    xarm-server (host, uv) ──► HDF5 episodes
                        │ stdin/stdout JSON, through `docker exec`
                        ▼
                    ros_bridge.py (container, ROS Python)
                        │ ROS topics + services
                        ▼
                    xarm_scheduled_node (container) ──► xArm over Ethernet
```

| Piece | Runs on | Where it lives |
|---|---|---|
| Quest client `xarm-vr` | host, uv | [quest_teleop/xarm_teleop/teleop.py](quest_teleop/xarm_teleop/teleop.py) |
| Server `xarm-server` | host, uv | [quest_teleop/xarm_teleop/server.py](quest_teleop/xarm_teleop/server.py) |
| ROS bridge | container | [quest_teleop/xarm_teleop/ros_bridge.py](quest_teleop/xarm_teleop/ros_bridge.py) |
| Bridge wrapper | host | [quest_teleop/ros_python_docker.sh](quest_teleop/ros_python_docker.sh) |
| Arm node | container | [xarm_scheduled_node.py](onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_ros2/xarm_scheduled_node.py) |
| Fake arm node | container | [fake_xarm_node.py](onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_ros2/examples/fake_xarm_node.py) |
| Arm and gripper controllers | container | [xarm_controller/](onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_controller/) |

The container exists because ROS 2 Humble only ships for Ubuntu 22.04 and this
host is 24.04. It uses host networking, so there are no ports to map: ROS nodes
inside it reach the arms on `192.168.1.x` directly. The host has no ROS, so
`ros2` commands only work inside the container.

## 2. One-time setup

### Host

- **Docker group.** You are in the `docker` group, but a terminal opened before
  that needs `newgrp docker`. Check with `docker ps`.
- **uv environment.** `cd quest_teleop && uv sync --locked`.
- **Quest.** `adb devices` must show the headset as `device`. In the headset,
  turn off *Auto Switch Between Hands and Controllers*.

### ROS-only container

`docker_ros/` is `docker_5090/` with CUDA, PyTorch and the ML packages
commented out. It needs no NVIDIA Container Toolkit.

```bash
cd ~/experiments/onrobot_rl_env/docker_ros
./build.sh        # first time, or after changing the Dockerfile / env yaml
./start.sh
./join.sh         # once per terminal
```

Inside the container, build the workspace once:

```bash
senv
cd /workspace/manipulation_ws
colcon build --symlink-install
source install/setup.bash
```

`./stop.sh` removes the container. The image and the workspace build
(`manipulation_ws/build`, `install`, `log`) survive; anything installed by hand
inside the container does not.

## 3. Dry run with the fake arm

Checks the whole chain, Quest to ROS topic, with no robot. Make sure no real arm
node is running on the same arm name.

**Container terminal 1: fake arm**
```bash
senv
python3 /workspace/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_ros2/examples/fake_xarm_node.py --ros-args -p arm_name:=left_arm
```

**Host terminal 1: server**
```bash
cd ~/experiments/quest_teleop
uv run xarm-server --backend ros --arm left --execute --ros-python ./ros_python_docker.sh
```

**Host terminal 2: Quest client**
```bash
cd ~/experiments/quest_teleop
uv run xarm-vr --arm left --controller auto --execute --show-input
```

**Container terminal 2: watch the stream**
```bash
senv
ros2 topic echo /left_arm/xarm_command_pose
ros2 topic hz /left_arm/xarm_command_pose     # about 50 Hz
```

While the side grip is held, `cmd` is six numbers (metres, then a rotation
vector in radians). When it is released, `cmd` is empty; that is the hold
message. The fake arm teleports to each command and has no limits, so this
checks the stream and axis mapping, not speeds or bounds.

## 4. Real arm

### Before starting

- Stop the fake node.
- The arm must already be inside the workspace bounds. The node refuses to start
  control otherwise and does not move the arm there for you.
- E-stop within reach. Keep the first motions small.

### Commands (left arm, `192.168.1.220`)

**Container: arm node**
```bash
senv
ros2 run xarm_ros2 xarm_scheduled_node --ros-args \
  -r __node:=xarm_left -p arm_name:=left_arm -p xarm_ip:=192.168.1.220 \
  -p enable_gripper:=true -p enable_keyboard:=false \
  -p lower_bounds:="[0.282, -0.226, 0.237]" \
  -p upper_bounds:="[0.524, 0.298, 0.590]" \
  -p tcp_offset:="[0.0, 0.0, 0.011, 0.0, 0.0, 0.0]" \
  -p max_vel_m_s:=0.1 -p max_ang_vel_rad_s:=0.6
```

**Host: server, then client** (same as the dry run)
```bash
uv run xarm-server --backend ros --arm left --execute --ros-python ./ros_python_docker.sh
uv run xarm-vr --arm left --controller auto --execute --show-input
```

Stop in this order: client, server, arm node. After any stop, restart the
server before the next session; on the ROS backend, ending control closes the
bridge.

### Variations

- **Right arm:** use `right` / `right_arm` / `xarm_right` and
  `xarm_ip:=192.168.1.225`.
- **Both arms:** run one node per arm, then `--arm both` on server and client.
  This needs two controllers.
- **No gripper:** `enable_gripper:=false` on the node, and `--no-gripper` on
  both the server and the client.

### Controls

| Action | Control |
|---|---|
| Move the arm | Hold side grip; each press re-zeros against the current TCP |
| Close gripper | Hold front trigger; release opens |
| Hold the arm | Release side grip |

## 5. Recording episodes

Add `--record --task "description"` to the `xarm-vr` command. Episodes are
written by the server to `quest_teleop/episodes/` as `.h5` files
(`.partial.h5` while recording).

Per arm, each file holds `observation/<arm>/state`,
`next_observation/<arm>/state`, `action/<arm>/requested`,
`action/<arm>/applied`, `action/<arm>/active` and `timestamp`. The writer is
[recording.py](quest_teleop/xarm_teleop/recording.py).

**No images are recorded on the real arm yet.** The physical backends return no
cameras ([ros_backend.py `cameras()`](quest_teleop/xarm_teleop/ros_backend.py),
[backends.py `RealBackend.cameras()`](quest_teleop/xarm_teleop/backends.py)).
A camera source has to be added there before vision datasets are possible.

## 6. Where each setting lives

This is the part that is easy to get wrong.

| Setting | Read by | File or flag |
|---|---|---|
| Workspace bounds, speed limits, TCP offset, payload | arm node | `-p` flags on `xarm_scheduled_node` (defaults in the node source) |
| Translation / rotation scale, button mapping, VR-to-robot axes, jump thresholds | `xarm-vr` client | [quest3.yaml](onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_controller/config/quest3.yaml) |
| Arm IPs, bounds, start pose for the direct SDK backend and example scripts | `--backend real`, examples | [roamlab.yaml](onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_controller/config/roamlab.yaml) |
| Launcher GUI arm settings | `ros_launcher` | [launcher_configs/](onrobot_rl_env/manipulation_ws/src/manipulation_env/launcher_configs/) |

With the ROS backend, bounds and speeds in `roamlab.yaml` and the `control:`
block of `quest3.yaml` are **not** used by the arm node. Pass them as `-p`
flags.

## 7. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `permission denied ... docker.sock` | Terminal lacks the `docker` group | `newgrp docker`, or log out and back in |
| `ROS bridge exited; check its stderr output` at server start | Wrapper could not reach Docker, or the container is not running | `docker ps`; `./start.sh` |
| `Timed out waiting for the ROS bridge` | No arm node (real or fake) answering on that arm name | Start the node first; check `arm_name` matches |
| `ROS feedback is stale or control is stopped` | Arm node stopped publishing, reported an error, or control stopped | See the checks below |
| `controller pose jumped; release buttons` | Quest tracking glitch: more than 15 cm or 0.8 rad between samples | Keep the headset still and the controller in view; restart server and client |
| Start refused: `TCP feedback is invalid or outside the workspace` | Arm is outside the node's bounds | Move the arm inside, or widen the `-p` bounds |
| `Temporary failure resolving` during image build | Docker's bridge network gets DNS servers that do not answer here | `docker_ros/build.sh` already builds with `--network=host` |
| `Could NOT find OpenCV (missing: calib3d)` in `colcon build` | conda installed OpenCV 5 | Pinned `<5` in the env yaml; rebuild the image |

Checks for the arm node, from a container shell after `senv`:

```bash
ros2 topic echo /left_arm/xarm_control_status --once --full-length
ros2 topic hz /left_arm/xarm_state            # about 60 Hz
ros2 topic hz /left_arm/xarm_gripper_state    # about 60 Hz with a gripper
```

The status JSON shows `arm_running`, `gripper_running` and `error`.

## 8. Changes made during setup

**onrobot_rl_env**, branch `ros-only-docker`

- Pushed: `docker_ros/` and `fake_xarm_node.py`.
- **Not committed yet:**
  - Control-loop fix in
    [xarm_scheduled_controller.py](onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_controller/xarm_scheduled_controller.py)
    and
    [xarm_gripper_scheduled_controller.py](onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_controller/xarm_gripper_scheduled_controller.py).
    The idle loop slept while holding the command lock, which starved the
    bridge's hold messages and froze the node's feedback. The sleep now happens
    outside the lock. Without this fix the real arm does not work over ROS.
  - Your edits to `quest3.yaml`, `roamlab.yaml` and
    `xarm_gripper_launcher.yaml`.

**quest_teleop**, branch `ros-docker-bridge`

- Pushed: `ros_python_docker.sh` and `run_sim.sh`.

## 9. Open items

- Commit and push the control-loop fix.
- `test_workspace_and_tcp_configuration_are_preserved` in
  [test_vr_teleop.py](onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_controller/tests/test_vr_teleop.py)
  fails because it expects the old left-arm bounds in `roamlab.yaml`.
- Add `server.log` to `quest_teleop/.gitignore` (`run_sim.sh` writes it).
- Add camera recording to the physical backends.
- The arm node speed is 0.1 m/s in the commands above; raise it once the setup
  is trusted.
- Policy deployment: a policy process can drive the same server through
  `ControlClient` in [vr_support/link.py](onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_controller/vr_support/link.py)
  (`claim`, `observation`, `command`). Not tried yet.

## 10. Further reading

**Docs**

- [quest_teleop/README.md](quest_teleop/README.md): the teleop design, controls, stop and hold behaviour, and the ROS section.
- [xarm_ros2/README.md](onrobot_rl_env/manipulation_ws/src/xarm_ros2/README.md): arm node topics, services and the hold/timeout protocol.
- [xarm_controller/README.md](onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_controller/README.md): the low-level controllers and tools.
- [vr_support/README.md](onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_controller/vr_support/README.md): headset setup and the Oculus Reader APK.
- [onrobot_rl_env/README.md](onrobot_rl_env/README.md) and [docker_5090/README.md](onrobot_rl_env/docker_5090/README.md): the lab's full stack and original Docker flow.
- [manipulation_env/README.md](onrobot_rl_env/manipulation_ws/src/manipulation_env/README.md): the lab's env node, launcher GUI and data collection (not used by this teleop path).

**Code, in the order a command travels**

1. [vr_support/oculus.py](onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_controller/vr_support/oculus.py): reads Quest poses and buttons over adb.
2. [vr_support/core.py](onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_controller/vr_support/core.py): clutch, taring, scaling and the pose-jump check.
3. [teleop.py](quest_teleop/xarm_teleop/teleop.py): the client loop that sends commands to the server.
4. [server.py](quest_teleop/xarm_teleop/server.py): control ownership, ticking and recording.
5. [ros_backend.py](quest_teleop/xarm_teleop/ros_backend.py) and [ros_bridge.py](quest_teleop/xarm_teleop/ros_bridge.py): the hand-off into ROS and the readiness rules.
6. [xarm_scheduled_node.py](onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_ros2/xarm_scheduled_node.py): topics, start/stop services, status and state publishing.
7. [xarm_scheduled_controller.py](onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_controller/xarm_scheduled_controller.py): bounds clipping, velocity limits, interpolation and the servo loop.

**Container files**

- [docker_ros/Dockerfile](onrobot_rl_env/docker_ros/Dockerfile), [conda_environment_docker_ros.yaml](onrobot_rl_env/docker_ros/conda_environment_docker_ros.yaml), [docker-compose.yml](onrobot_rl_env/docker_ros/docker-compose.yml), [source_env.sh](onrobot_rl_env/docker_ros/source_env.sh) (what `senv` runs).
