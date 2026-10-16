# xArm Quest teleop on roam03: setup and run guide

How to drive the xArm from a Quest 3 on this machine. ROS runs in a container
(`onrobot_rl_env/docker_ros`); the Quest client runs on the host under uv.
Both repositories sit side by side in `~/experiments`.

## 1. How the pieces fit

```text
Quest 3 ──USB/adb──► xarm-vr (host, uv)                      quest_teleop
                        │ VR link: JSON over tcp://127.0.0.1:8765
                        ▼
                    run_env + QuestAgent (container)          onrobot_rl_env
                        │ ROS topics + start/stop services
                        ▼
                    xarm_scheduled_node (container) ──► xArm over Ethernet
```

- **xarm-vr** owns everything VR-specific: ADB, the Quest app, taring on each
  grip press, scaling, axis and button mapping. It sends absolute TCP targets,
  gripper pulses, button values and named events (A/X = `episode`, long
  B/Y = `discard`).
- **QuestAgent** is the robot side. It serves the VR link, executes the newest
  fresh command each control tick, holds otherwise, and tells the client
  whether it is executing. While it is not (agent inactive, arm node not
  ready), the client holds and re-tares when control resumes.
- **xarm_scheduled_node** owns the xArm connection, bounds and speed limits.
  If commands stop for 0.3 s it holds the arm.

| Piece | Runs on | Code |
|---|---|---|
| Quest client `xarm-vr` | host, uv | [teleop.py](../xarm_teleop/teleop.py) |
| VR link protocol | both | [vr_support/link.py](../../onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_controller/vr_support/link.py) |
| Quest agent | container | [quest_agent.py](../../onrobot_rl_env/manipulation_ws/src/manipulation_env/manipulation_env/agents/quest_agent.py) |
| run_env (env node, data collection) | container | [run_env.py](../../onrobot_rl_env/manipulation_ws/src/manipulation_env/manipulation_env/run_env.py) |
| Arm node | container | [xarm_scheduled_node.py](../../onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_ros2/xarm_scheduled_node.py) |
| Fake arm node | container | [fake_xarm_node.py](../../onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_ros2/examples/fake_xarm_node.py) |

The container exists because ROS 2 Humble ships for Ubuntu 22.04 and this host
is 24.04. It uses host networking, so the arms (`192.168.1.x`) and the VR link
port need no mapping. The `onrobot_rl_env/scripts/` wrappers run everything in
it from the host; you do not need `join.sh` or `senv` for normal use.

## 2. One-time setup

### Host

- **Docker group.** `docker ps` must work without sudo. A terminal opened
  before you joined the group needs `newgrp docker`.
- **uv.** `curl -LsSf https://astral.sh/uv/install.sh | sh`, then
  `cd ~/experiments/quest_teleop && uv sync --locked`.
- **adb.** `sudo apt install adb`. Ubuntu's rules do not cover Meta's USB
  vendor ID, so add one, then replug the headset:

  ```bash
  echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2833", MODE="0660", GROUP="plugdev", TAG+="uaccess"' \
    | sudo tee /etc/udev/rules.d/52-meta-quest.rules
  sudo udevadm control --reload-rules && sudo udevadm trigger && adb kill-server
  ```

- **Cable.** Use a short data cable straight into the PC. A flaky cable shows
  up as `usb 1-1: USB disconnect` in `journalctl -kf` and ends the session.

### Headset

- Developer mode on; accept **Allow USB debugging** (tick *Always allow*).
- The teleop APK `com.rail.oculus.teleop` at version `1.1-teleopfix` (see the
  [headset guide](../../onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_controller/vr_support/README.md)).
- Turn off **Auto Switch Between Hands and Controllers**.

### Container

```bash
cd ~/experiments/onrobot_rl_env
docker_ros/build.sh      # first time, or after changing the Dockerfile / env yaml
./scripts/build.sh       # colcon build of the workspace (starts the container if needed)
```

`docker_ros/stop.sh` removes the container. The image and the workspace build
survive; anything installed by hand inside the container does not.

## 3. Run

Three terminals. Start the robot side first; the client waits for it.

```bash
# 1. Arm node (settings from roamlab.yaml)
cd ~/experiments/onrobot_rl_env && ./scripts/arm.sh left

# 2. run_env with the Quest agent
cd ~/experiments/onrobot_rl_env && ./scripts/run_env.sh

# 3. Quest client: turns the proximity sensor off, then connects
cd ~/experiments/quest_teleop && ./run_robot.sh
```

Until the agent starts, the client prints `Robot side is not executing: agent
inactive` and holds. Press A/X on the controller to start a demo (this starts
the agent), or `[a]` in run_env to drive without recording. Starting the agent
calls `xarm_start_control` on the node. The node
refuses if the arm is outside its workspace bounds or reports a fault, and
never moves the arm to a start pose by itself.

**Dry run without the robot:** `./scripts/arm.sh left --fake` instead of
step 1. The fake node accepts the same services and reports each commanded
pose as the actual pose, with no limits.

**Stop:** Ctrl+C in the client, `[q]` in run_env, Ctrl+C in the arm node. The
arm holds as soon as commands stop. Closing a terminal also stops what it
started in the container.

### What the scripts do

| Script | Does |
|---|---|
| `onrobot_rl_env/scripts/arm.sh left\|right [--fake] [-p name:=value]` | Starts `xarm_scheduled_node` (or the fake node) with `enable_keyboard:=false`; refuses a second node for the same arm. Extra `-p` flags override the config. |
| `onrobot_rl_env/scripts/run_env.sh [config] [run_env options]` | Rebuilds `manipulation_env` (picks up new env configs), then runs `run_env`. Default config: `xarm_quest/teleop_left`. |
| `onrobot_rl_env/scripts/build.sh [packages]` | `colcon build --symlink-install` in the container. |
| `quest_teleop/quest_prep.sh [--restore]` | Checks the headset and APK, turns the proximity sensor off (`--restore` turns it back on). Called by the run scripts. |
| `quest_teleop/run_robot.sh [left\|right\|both] [xarm-vr options]` | `quest_prep.sh`, waits for the Quest agent, runs `xarm-vr --execute --show-input`. |
| `quest_teleop/run_sim.sh [arm] [xarm-vr options]` | `quest_prep.sh`, the MuJoCo simulator and the client in one terminal. |

The proximity sensor stays off until the headset reboots, so it stays awake
and tracks on a stand. This is the same `prox_close` broadcast that MQDH sends.

## 4. Controls

| Action | Control |
|---|---|
| Move the arm | Hold the side grip. Each press re-zeros against the current TCP. |
| Close / open the gripper | Hold / release the front trigger. |
| Hold the arm | Release the side grip. |
| Start a demo, or stop and save it | Press A (right controller) or X (left) |
| Discard the latest demo | Hold B or Y for 1 s: discards the demo being recorded, or the last saved one when idle |

The headset buttons go to the same handlers as run_env's keys. A/X starts the
agent if needed, so you do not have to press `[a]` first. After a stop or a
discard the agent stays active, so you can keep moving the arm to reset the
scene. The arm holds while A/X/B/Y is pressed; after a press, release the grip
before moving again. The hold time is `discard_hold_s` in `quest3.yaml`.

run_env keys: `[a]` start the agent (arm streams), `[b]` start recording a demo
(also starts the agent), `[c]` stop the agent and save the demo, Backspace
delete the last demo (asks y/n), `[q]` quit.

## 5. Simulator

```bash
cd ~/experiments/quest_teleop
./run_sim.sh                 # right arm, MuJoCo viewer
./run_sim.sh left --record --task "Pick up the cube"
./run_sim.sh right --demo --seconds 5    # no headset
```

A/X saves the current HDF5 episode, resets the scene and starts the next one.
Holding B/Y does nothing in the simulator.
Server output goes to `server.log`. See the [README](../README.md) for the
simulator, the HDF5 format and the Gymnasium environment.

## 6. Where each setting lives

| Setting | Read by | File |
|---|---|---|
| Arm IP, workspace bounds, speed limits, TCP offset, payload, gripper on/off | arm node | [roamlab.yaml](../../onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_controller/config/roamlab.yaml), section `left` / `right`; `-p` flags override |
| Translation / rotation scale, buttons, VR-to-robot axes, jump thresholds | `xarm-vr` | [quest3.yaml](../../onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_controller/config/quest3.yaml) (`defaults`, `arms`) |
| Which arms and grippers the agent drives, link endpoint, cameras, control rate, data folder | run_env | [xarm_quest/teleop_left.yaml](../../onrobot_rl_env/manipulation_ws/src/manipulation_env/env_configs/xarm_quest/teleop_left.yaml) |
| Arm settings for the lab's launcher GUI | `ros_launcher` | [launcher_configs/](../../onrobot_rl_env/manipulation_ws/src/manipulation_env/launcher_configs/) (passes every value explicitly) |

The `control:` block of `quest3.yaml` only feeds the direct-SDK example script;
the arm node's speed limits are in `roamlab.yaml`.

## 7. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `The Quest app did not start within 10 s` | Headset locked or showing a dialog | Put it on, unlock it, rerun |
| `No USB permission` / `no permissions` in `adb devices` | udev rule missing | Section 2, then replug |
| `unauthorized` | USB debugging not accepted | Put the headset on and accept |
| Session ends with `ADB log stream ended` | USB link dropped | Check the cable; `journalctl -kf \| grep usb` |
| `APK reports no tracked controllers` while starting | Controllers asleep or out of view | Wake the controller and keep it in view |
| Client holds: `agent inactive` | run_env agent not started | Press A/X on the controller, or `[a]` in run_env |
| Client holds: `arm stream is not running` / `no recent xarm_state feedback` | Arm node stopped, not started, or faulted | Check the node terminal; restart `./scripts/arm.sh` and press `[a]` |
| `Could not start the arm streams` in run_env | Node refused start: outside workspace or robot error | Move the arm inside the bounds or clear the error, then `[a]` |
| `run_robot.sh` keeps waiting for the agent | run_env not running, or its config is not `quest` | `./scripts/run_env.sh` |
| `An arm node for left_arm is already running` | A node is still up | Stop it (its terminal, or `docker_ros/stop.sh`) |
| `controller pose jumped; release buttons` | Tracking glitch: >15 cm or >0.8 rad between samples | Keep the controller in view; restart the client |
| `permission denied ... docker.sock` | Terminal lacks the docker group | `newgrp docker` |
| `No module named 'click'` in run_env | Image built before `click` was added | `docker_ros/build.sh` |

Arm node checks, from `docker_ros/join.sh` then `senv`:

```bash
ros2 topic echo /left_arm/xarm_control_status --once --full-length
ros2 topic hz /left_arm/xarm_state            # about 60 Hz
ros2 topic hz /left_arm/xarm_command_pose     # about 20 Hz while run_env's agent runs
```

## 8. Open items

- Settle the training data format for the recorded demos.
- The right arm has a DITTO hand, not a gripper; there is no Quest config for it.
- Rebuild the container image to pick up `click` (`docker_ros/build.sh`).

## 9. Code, in the order a command travels

1. [vr_support/oculus.py](../../onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_controller/vr_support/oculus.py): reads Quest poses and buttons over ADB.
2. [vr_support/core.py](../../onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_controller/vr_support/core.py): clutch, taring, scaling and the pose-jump check.
3. [teleop.py](../xarm_teleop/teleop.py): the client loop.
4. [vr_support/link.py](../../onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_controller/vr_support/link.py): the VR link, client and server halves.
5. [quest_agent.py](../../onrobot_rl_env/manipulation_ws/src/manipulation_env/manipulation_env/agents/quest_agent.py): link to run_env actions, readiness.
6. [robot_arm.py](../../onrobot_rl_env/manipulation_ws/src/manipulation_env/manipulation_env/robots/robot_arm.py): arm topics, start/stop services, status checks.
7. [xarm_scheduled_node.py](../../onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_ros2/xarm_scheduled_node.py): node services, status and state publishing.
8. [xarm_scheduled_controller.py](../../onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_controller/xarm_scheduled_controller.py): bounds clipping, velocity limits, interpolation and the servo loop.
