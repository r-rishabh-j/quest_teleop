# xArm teleoperation and simulation

Use a Quest 3 over **USB** to drive the xArm, either in a Gymnasium/MuJoCo
simulator (this repo) or on the real arms through `onrobot_rl_env`. The client
can control the left arm, the right arm, or both.

```text
Quest 3 + Oculus Reader APK
          │ USB / ADB
          ▼
      xarm-vr client            (taring, scaling, axis and button mapping)
          │ VR link: length-prefixed JSON (vr_support/link.py)
          ├─ unix socket ─► xarm-server (sim) → MuJoCo 3 → HDF5 episodes
          └─ tcp 8765 ────► run_env QuestAgent (onrobot_rl_env, ROS container)
                              → xarm_scheduled_node → xArm; data via DataCollector
```

The VR link uses small JSON messages with a four-byte length prefix over one
persistent connection. The simulator listens on a Unix socket; run_env's Quest
agent on `tcp://127.0.0.1:8765` (the ROS container uses host networking). For
the real robot, see [docs/TELEOP_SETUP.md](docs/TELEOP_SETUP.md).

## Working baseline and next cleanup

On 2026-10-12, Quest teleoperation worked after disabling **Auto Switch Between
Hands and Controllers** in the headset. Keep this off for controller teleop.
The first-clutch pose drop may have come from that input-mode switch.

Keep the approach small:

- Read timestamped Quest poses and buttons over USB/ADB. Bind each arm to one
  controller at startup; never switch hands during the session.
- Each side-grip press tares controller pose against the current arm TCP.
  Apply relative movement only while held. Front trigger closes the gripper;
  release opens it. B/Y explicitly resets the simulation episode.
- Missing poses or samples older than 0.25 seconds hold all selected arms and
  grippers. Keep the client and episode open. Re-tare on the first fresh input,
  without a recovery timer or replaying movement from the gap.
- Keep input handling separate from the robot side. Use the same VR link for
  the MuJoCo simulator and for run_env's Quest agent on the real arms.

Next cleanup: consolidate input validation and hold/re-tare handling; remove
duplicate checks, superseded workarounds, and unused APK demo features. Preserve
tests for clutch taring, fixed hand mapping, gripper behavior, stale-input
recovery, and explicit episode resets. Keep transport and robot fault handling.

## Install the uv environment

Choose any parent folder and place the repositories beside each other with
these directory names. The parent folder can have any name or location.

```text
workspace/                       # any parent folder
  onrobot_rl_env/
    manipulation_ws/src/xarm_ros2/
      xarm_ros2/xarm_controller/  ← shared controller source
  xarm_teleop/                    ← this project
  oculus_reader/                  # optional: patched Quest APK source
```

From your chosen parent folder:

```bash
git clone https://github.com/roamlab/onrobot_rl_env.git onrobot_rl_env
git clone https://github.com/r-rishabh-j/quest_teleop.git xarm_teleop
cd xarm_teleop
uv sync --locked
```

If the repositories are already present, arrange them as shown, enter
`xarm_teleop`, and run `uv sync --locked`. The local dependency uses
`../onrobot_rl_env/...`; no machine-specific absolute path is required.
The xArm source is included in the environment repository, so this setup does
not require a separate `xarm_ros2` or `xarm_controller` checkout.

Run the following commands from **`xarm_teleop` in each terminal**, unless a
section explicitly changes directories. The optional `oculus_reader` folder
is needed only for the local APK build guide, not for `uv sync`.

This creates `.venv`. The checked-in lock uses Python 3.11, MuJoCo 3.8.0,
Gymnasium 1.2.3, NumPy, SciPy, PyYAML, Pillow, and h5py. The `xarm_controller`
source inside `onrobot_rl_env` is an editable dependency shared with its ROS
nodes. Edit the controller there. The simulation uses the xArm XML and meshes from
`gym-xarm`; it has its own Python environment wrapper. Asset source notes and
the license are in [`assets/gym_xarm`](xarm_teleop/assets/gym_xarm/README.md).

The uv environment does not use ROS. The real arms are driven from the ROS
container in `onrobot_rl_env`; this repo only runs the Quest client for them.

## Mac: start the simulator

For one Quest controller, use this order. After updating Python code, stop the
old client and server with Ctrl+C, then start both again. These button/session
changes do not require another APK rebuild.

1. Connect the Quest by USB, open MQDH, and turn off the proximity sensor.
   In the headset, turn off **Auto Switch Between Hands and Controllers**.
   Keep MQDH connected, wake the controller, and keep the headset still.
2. Confirm the USB device and corrected app version:

```bash
adb devices
adb shell dumpsys package com.rail.oculus.teleop | grep versionName
```

The device must show `device`, and the app must show `1.1-teleopfix`. If the APK
needs installation, use the [build/install guide](../oculus_reader/app/patches/README.md).
The built file is `OculusTeleop-debug.apk`, not the bundled beta APK.

3. Start the simulation in Terminal 1:

```bash
uv run xarm-server --arm right --viewer
```

`xarm-server` launches **mjpython on macOS**. The launcher also sets the Python
shared-library path needed by uv-managed Python. On Linux it uses normal Python.

4. Start the real Quest input in Terminal 2:

```bash
uv run xarm-vr --arm right --controller auto --show-input
```

This command starts the Quest app automatically. Use `--no-launch` only when
you already opened the app and want the client to attach without launching it.
In both cases, the client force-stops the app when the client exits.

5. Release the side grip and front trigger. Hold the side grip to move the arm.
   Hold the front trigger to close the gripper; release it to open. Press B to
   reset the simulation and start a new episode. A left controller uses Y.
6. To stop, press Ctrl+C in Terminal 2, then Terminal 1. A closed ADB connection
   also ends the session. The client stops robot control first, then stops
   the Quest app. Old samples or a tracking gap hold motion and wait for the
   same controllers to return. This does not reset the scene or start an episode.

The client also stops the app after a startup error or when `--seconds` ends.
If the USB connection is lost, it reports that app cleanup failed. Close the
app on the headset in that case. A forced process kill (`kill -9`) cannot run
this cleanup. Pressing B/Y resets the simulation; it leaves the Quest app running.

For a software-only test without a headset, use this client instead:

```bash
uv run xarm-vr --demo --arm right --seconds 5 --show-input
```

The demo moves the arm a short distance and closes/reopens the gripper. To select both:

```bash
# Terminal 1
uv run xarm-server --arm both --viewer

# Terminal 2
uv run xarm-vr --demo --arm both --seconds 5
```

Each simulated arm has its own lift scene. The two scenes do **not** share
objects or collisions. The server API supports both arms, but a shared bimanual
physics scene is a separate extension.
The viewer shows one selected scene. Use `--view-arm right` or `--view-arm left`
to choose it. Both scenes continue to run and record.

For a state-only test without graphics:

```bash
uv run python -m xarm_teleop.server --no-images
uv run xarm-client smoke --seconds 3 --record
```

On a Linux machine without a display, set `MUJOCO_GL=egl` for RGB rendering if
an EGL driver is available. Use `--no-images` when no graphics backend is present.

## Quest input over USB

The Quest must run an Oculus Reader APK. It is not included in this project.
Enable developer mode, authorize USB debugging, and install the Quest 3 APK.
See the [headset setup guide](../onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_controller/vr_support/README.md), including
the tracking-flag defect found in the current Quest 3 fork and its required fix.

```bash
adb devices
uv run xarm-vr --arm right --show-input
```

One controller is enough for one robot arm. `--arm` selects the **robot**;
`--controller` selects the **physical Quest hand**. For one arm, the default
`--controller auto` uses the available tracked controller. If both are available,
it uses the hand configured for that arm. The selection is fixed for the run;
losing that controller holds motion; it never switches to the other hand.

```bash
# Either single controller can drive the right robot arm.
uv run xarm-vr --arm right --controller auto --show-input

# Require the right controller, including at startup.
uv run xarm-vr --arm right --controller right --show-input

# Explicitly use the right hand controller for the left robot arm.
uv run xarm-vr --arm left --controller right --show-input
```

The grip, front trigger, and episode buttons follow the physical hand: right uses
`RG`, `RTr`, and B; left uses `LG`, `LTr`, and Y. Custom button roles also follow the
hand. The resolved mapping is printed and stored in episode metadata.
`--arm both` still needs two controllers. It does not duplicate one hand's motion
onto both robots.

Use `--serial USB_SERIAL` if more than one ADB device is attached. The default
source uses the attached USB device. This workflow does not enable ADB Wi-Fi.
The script starts the installed app and reads its timestamped `logcat` records.
On exit, it sends `adb shell am force-stop com.rail.oculus.teleop` to the same
device. The next normal client start launches the app again.

If the app closes on launch, read `adb logcat -b crash -d -t 120`. A crash with
`XrApp::HandleSessionStateChanges: assertion "Resumed" failed` is in the APK's
Meta sample framework. The Python timeout is a consequence of the app exiting.
The [local APK build fix](../oculus_reader/app/patches/README.md) addresses that
event-order assertion and the per-hand tracking guards; it requires rebuilding
and installing the APK.

As an unverified workaround, open the app manually while the headset is awake
and worn. Then attach without another Android launch request:

```bash
uv run xarm-vr --arm right --controller auto --show-input --no-launch
```

Keep MQDH connected with the proximity sensor off before removing the headset.
If the app also crashes on manual launch, `--no-launch` cannot fix that APK.
Startup diagnostics now distinguish a missing app process, no pose records,
invalid records, and a missing controller.
They also show sample age and distinguish `APK reports no tracked controllers`
from `APK hand mismatch`. The latter is the beta APK's wrong-hand packet defect.
Auto-selection cannot repair a pose that the APK did not transmit. Rebuild and
install the corrected APK; do not relabel an untracked hand's identity matrix.

Default controls:

| Function | Left controller | Right controller |
|---|---|---|
| Hold to enable arm motion | Side grip | Side grip |
| Close gripper | Hold front trigger (`LTr`) | Hold front trigger (`RTr`) |
| Open gripper | Release front trigger | Release front trigger |
| Hold the arm | Release grip | Release grip |
| Reset simulation and start a new episode | Y | B |

**Each press sets a new zero point.** The client captures the controller pose and
the arm TCP pose at that press. It then applies only the controller's relative
movement. Release, reposition your hand, and press again to set a new reference.

B/Y is edge-triggered: holding it causes one reset, not repeated resets. It must
be released before another reset can occur. After reset, release the side grip
and front trigger before using them. The controller selection remains fixed.
The scene resets with the server's `--seed`, so reset does not choose a new seed.
There is no automatic reset or new episode after a disconnect, timeout, or success.

The headset may stay on a stand with its wear sensor disabled through Meta
Quest Developer Hub. Keep it still while a movement button is held. The APK
reports controller poses relative to the headset. Taring removes the initial
offset; it cannot remove later headset motion. Verify tracking without wearing
the headset before enabling a physical robot.

Change buttons, scale, gripper behavior, and each arm's axis mapping in
[`quest3.yaml`](../onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_controller/config/quest3.yaml), or pass `--vr-config PATH`.
Use `--arm left`, `right`, or `both`. With both selected, each grip independently
enables its arm. `--no-gripper` disables gripper commands from the client.

## Stops and episode boundaries

A missing pose from **any selected controller**, no sample, or a sample older
than `stale_timeout_s` (0.25 seconds) holds both selected arms and grippers.
This pauses motion; it does **not** close the client, app, or episode. The client
sends hold commands while it waits. The **first fresh valid pose from the same
controllers** clears the old motion origins. If the side grip is still held,
its current pose and the current
arm TCP become the new zero point. Movement starts with the next valid sample;
there is no extra recovery delay.
Motion made during the gap is discarded. The controller-to-arm mapping never
changes during a session, including with `--controller auto`.

The gripper holds its position through tracking loss and recovery. Release the
front trigger, then press it again to use the gripper. A B/Y press made during
the gap cannot start a new episode. With both arms selected, both controllers
must recover before either arm can move.

Use the normal client command:

```bash
uv run xarm-vr --arm right --controller right --show-input
```

The previous `--tracking-recovery` timer and option were removed. They added a
delay after tracking had already returned. A grip press does not start a timer.

The 0.25-second setting is a motion pause threshold, not an exit timeout. A
delayed control loop also discards its old origin before accepting fresh input.
No episode reset, controller switch, or extra recovery timer is involved.

A closed USB/ADB connection, invalid pose/button data, an invalid clock, or a
pose jump during active control still ends the session. Invalid
packets report their parse error; they are not labelled as missing tracking.
After a stop, the client does not reconnect, reclaim control, or reset the
environment by itself.
It reports the actual sample age and limit when old data pauses motion. `--show-input`
also shows the actual age while the controls are released.

Simulation physics pauses when no client owns control. Interactive simulation
has no 60-second cutoff and does not end when the cube is lifted. B/Y is the
explicit new-episode/reset action. After a fault, restart the client yourself;
it must see released controls before accepting motion.

For a bounded evaluation, opt in with `--episode-steps 3000` and/or
`--stop-on-success`. These options stop at the boundary; they never reset or
start the next episode automatically. Direct Gymnasium use keeps its normal
termination/truncation behavior.

The server now reports the actual stop reason, such as `command timeout` or
`simulation time limit`. Cleanup after an ended session is safe and repeatable.
The old misleading `Lease belongs to another connection` error is fixed for
expired/ended sessions. Check the latest reason with:

```bash
uv run xarm-client health
```

The 0.3-second command watchdog remains enabled. For a graphics-related timeout,
`--no-images` disables off-screen RGB capture while `--viewer` still shows the
scene. That run will not record RGB images.
The Python API's `stop_recording()` also ends control. Start the next control
session explicitly; stopping a recording does not leave motion running between files.

## Real robot

The real arms are driven by `run_env`'s Quest agent in `onrobot_rl_env`, not by
`xarm-server`. Start the arm node and run_env there, then `./run_robot.sh` here.
[docs/TELEOP_SETUP.md](docs/TELEOP_SETUP.md) has the full guide. On the robot,
B/Y sends an `episode` event to run_env instead of resetting a simulation.

## Record demonstrations

```bash
# Server: choose the output directory.
uv run xarm-server --arm right --viewer --episodes ./episodes

# Client: B saves the current episode, resets the scene, and starts the next.
uv run xarm-vr --arm right --controller auto --record --task "Pick up the cube" --show-input
```

The first recording starts when the client obtains control. Each B/Y press ends
the current recording **before** reset and starts a new file **after** reset.
Ctrl+C or a fault saves the current file and stops; neither starts another file.
Without `--record`, B/Y still resets the simulation but creates no dataset file.

Files are written as `.partial.h5` while recording, then renamed to `.h5` after
a normal stop, disconnect, or timeout. Check `success`, `stop_reason`, `frames`,
and `complete` before using an episode for training. An operator stop defaults
to `success=false`; a policy/data-collection client can call `stop(success=True)`.
Demo and smoke-test episodes are marked as synthetic in their metadata or task.

Each server tick stores a transition. For each arm:

| Dataset | Shape | Meaning |
|---|---|---|
| `observation/<arm>/state` | T × 14 | TCP xyz, TCP rotvec, 7 joints, gripper pulse |
| `next_observation/<arm>/state` | T × 14 | Feedback after that tick |
| `action/<arm>/requested` | T × 7 | Requested absolute TCP pose and gripper pulse |
| `action/<arm>/applied` | T × 7 | Limited sim target, or sampled robot command feedback |
| `action/<arm>/active` | T × 2 | Arm and gripper command enabled |
| `observation/images/<arm>/rgb` | T × H × W × 3 | Simulated uint8 RGB, if enabled |
| `reward/<arm>`, `done/<arm>` | T | Task result |

The file also stores sample times, per-arm observation times, simulated time,
source sequence, software versions, selected arms, and the VR configuration.
Images belong to the observation **before** the corresponding action.

These HDF5 files come from the simulator only. Real-robot demonstrations are
recorded by run_env's DataCollector in `onrobot_rl_env`.

Read an episode without a training-framework dependency:

```python
import h5py

with h5py.File("episodes/episode_....h5") as episode:
    state = episode["observation/right/state"][:]
    action = episode["action/right/requested"][:]
    rgb = episode["observation/images/right/rgb"][:]  # simulated RGB
```

This is a documented HDF5 format, not a LeRobot dataset. Keep the recorded arm
order, units, and frame definition in your training configuration. Absolute
pose actions and rotation vectors must not be treated as normalized velocities
or Euler angles.

## Use Gymnasium or a learned policy

```python
import gymnasium as gym
import xarm_teleop.env  # registers the task
import numpy as np

env = gym.make("XArmTeleopLift-v0")
obs, info = env.reset(seed=0)
# Gym action: base-frame xyz (m), rotvec (rad), opening in [0,1].
action = np.r_[obs["tcp_pose"], 1.0]
obs, reward, terminated, truncated, info = env.step(action)
env.close()
```

The network API uses gripper pulses in `[0,850]` to match `xarm_controller`.
The simulation adapter converts pulses to opening fractions.

```python
from vr_support.link import ControlClient

client = ControlClient()  # or endpoint="tcp://127.0.0.1:8765"
client.claim()           # allow_real=True is required for physical backends
try:
    obs = client.observation(images=True)
    # Replace with your policy output: six pose values plus gripper pulse.
    target = obs["arms"]["right"]["tcp_pose"]
    client.command(obs, {"right": {"pose": target, "gripper": 850.0}})
finally:
    client.stop()
    client.close()
```

Request a new observation before each command. Its one-use ticket prevents
replayed or delayed commands from becoming current. For policy inference slower
than 0.3 seconds, separate inference from a local control loop; do not keep
resending old motion simply to defeat the timeout.

## Code map and checks

| File | Responsibility |
|---|---|
| `env.py` | Gym task, frame conversion, gripper control, MuJoCo stepping |
| `backends.py` | Simulation adapter for the server |
| `../onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_controller/vr_support/link.py` | VR link protocol: framing, `ControlClient`, request validation, and the `LinkSession` used by manipulation_env's QuestAgent |
| `server.py` | Control ownership, freshness checks, stepping, episode lifetime |
| `teleop.py` | Quest input to the shared relative-motion policy |
| `recording.py` | Observation/action pairing and HDF5 writes |
| `../onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_controller/vr_support/core.py` | Per-arm button state and taring |

```bash
uv run python -m unittest discover -s tests -v
uv run python -m unittest discover \
  -s ../onrobot_rl_env/manipulation_ws/src/xarm_ros2/xarm_ros2/xarm_controller/tests -v
```

Set `XARM_TEST_RENDER=1` and run the tests with `mjpython` in a desktop session
to include the RGB test. Tests cover state-only physics, a scripted cube lift,
command validation, ownership, timeouts, recording, and the xArm node's hold and
workspace callbacks. The live socket test skips if listening sockets are blocked.
Test physical xArm control through `onrobot_rl_env` on the robot machine.

The simulator uses a mocap weld, not calibrated motor dynamics. Workspace and
speed limits do not provide collision avoidance. Button release and timeouts
are software holds; the physical robot stop remains necessary.
