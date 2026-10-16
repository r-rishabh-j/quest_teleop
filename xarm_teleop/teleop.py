"""Quest client for the VR link. The server is the simulator or manipulation_env's QuestAgent."""
from __future__ import annotations

import argparse
from dataclasses import replace
from pathlib import Path
import time

import numpy as np

from vr_support.core import RelativeTeleop, TeleopConfig, VRFrame, valid_transform
from vr_support.oculus import AdbOculusSource
from vr_support.link import DEFAULT_ENDPOINT, ControlClient
from vr_support.runtime import CONFIG_DIR, load_yaml


class ControllerInputError(RuntimeError):
    pass


def bind_controller(config, controller):
    """Keep the configured button roles when using the opposite physical hand."""
    if controller == config.controller:
        return config
    pairs = [("LG", "RG"), ("LTr", "RTr"), ("LJ", "RJ"), ("X", "A"), ("Y", "B"),
             ("leftGrip", "rightGrip"), ("leftTrig", "rightTrig")]
    source = 0 if config.controller == "l" else 1
    target = 0 if controller == "l" else 1
    mapping = {pair[source]: pair[target] for pair in pairs}
    return replace(config, controller=controller,
                   move_button=mapping.get(config.move_button, config.move_button),
                   gripper_button=mapping.get(config.gripper_button, config.gripper_button),
                   episode_button=mapping.get(config.episode_button, config.episode_button),
                   discard_button=mapping.get(config.discard_button, config.discard_button))


def choose_controller(available, preferred):
    """Choose once at startup. Never switch hands automatically during control."""
    available = set(available) & {"l", "r"}
    if preferred in available:
        return preferred
    if len(available) == 1:
        return next(iter(available))
    raise ValueError("No usable controller is available")


def demo_frame(sequence, now, elapsed):
    phase = elapsed % 4
    poses = {"l": np.eye(4), "r": np.eye(4)}
    for pose in poses.values():
        pose[2, 3] = -0.04 * np.clip(phase - 0.5, 0, 1.5)
    return VRFrame(sequence, now, poses,
                   {"LG": float(0.5 <= phase < 2), "RG": float(0.5 <= phase < 2),
                    "LTr": float(2.5 <= phase < 2.7), "RTr": float(2.5 <= phase < 2.7),
                    "X": 0.0, "A": 0.0, "Y": 0.0, "B": 0.0})


class EventButtons:
    """Operator events from the face buttons of the bound controllers.

    A press of an episode button sends "episode"; holding a discard button for
    discard_hold_s sends "discard" once per hold. A button already down at
    startup or after an input gap is not a press.
    """

    def __init__(self, configs):
        self.configs = list(configs)
        self.hold_s = max(c.discard_hold_s for c in self.configs)
        self.reset()

    def reset(self):
        self._episode_down = None
        self._discard_since = None
        self._discard_sent = True  # a hold already in progress cannot discard

    def update(self, frame, now):
        """Returns (events, held): held is True while any event button is down."""
        def down(attr):
            return any(frame.buttons[getattr(c, attr)] >= c.button_threshold for c in self.configs)
        episode, discard = down("episode_button"), down("discard_button")
        events = []
        if self._episode_down is not None and episode and not self._episode_down:
            events.append("episode")
        self._episode_down = episode
        if not discard:
            self._discard_since, self._discard_sent = None, False
        else:
            if self._discard_since is None:
                self._discard_since = now
            if not self._discard_sent and now - self._discard_since >= self.hold_s:
                events.append("discard")
                self._discard_sent = True
        return events, episode or discard


def raw_buttons(frame):
    """Controller values sent with each command, for logging on the robot side."""
    if frame is None:
        return {}
    return {name: float(value) for name, value in frame.buttons.items()
            if isinstance(value, (int, float)) and np.isfinite(value)}


def input_pause_reason(frame, policies, now):
    """Old or missing input pauses motion. Corrupt input remains an error."""
    if frame is None:
        return "Waiting for a controller sample"
    if frame.error:
        raise ControllerInputError(f"Invalid Quest packet: {frame.error}; stopping all arms and grippers")
    missing = []
    for name, policy in policies.items():
        config = policy.config
        age = now - frame.captured_at
        if not np.isfinite(age) or age < 0:
            raise ControllerInputError(
                f"Invalid controller sample age={age:.6f}s; limit={config.stale_timeout_s:.3f}s; "
                f"sequence={frame.sequence}; stopping all arms and grippers"
            )
        if age > config.stale_timeout_s:
            return f"Sample age={age:.6f}s exceeds {config.stale_timeout_s:.3f}s; waiting for fresh input"
        if config.controller not in frame.poses:
            missing.append(config.controller)
            continue
        try:
            valid_transform(frame.poses[config.controller])
        except (ValueError, TypeError) as exc:
            raise ControllerInputError(f"Invalid {name} controller pose; stopping all arms and grippers") from exc
        required = [config.move_button, config.episode_button, config.discard_button]
        if config.gripper_enabled:
            required.append(config.gripper_button)
        for button in required:
            if button not in frame.buttons or not np.isfinite(frame.buttons[button]):
                raise ControllerInputError(f"Invalid controller button {button}; stopping all arms and grippers")
    return f"APK has no pose for {missing}" if missing else None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server", default=DEFAULT_ENDPOINT)
    parser.add_argument("--arm", choices=("left", "right", "both"), default="right")
    parser.add_argument("--controller", choices=("auto", "left", "right"), default="auto",
                        help="Physical Quest controller for one selected robot arm; auto uses the available hand")
    parser.add_argument("--vr-config", type=Path, default=CONFIG_DIR / "quest3.yaml")
    parser.add_argument("--serial")
    parser.add_argument("--adb", default="adb")
    parser.add_argument("--no-launch", action="store_true", help="Read the app you opened manually; the app is still stopped on client exit")
    parser.add_argument("--demo", action="store_true", help="Synthetic controller input, simulation only")
    parser.add_argument("--execute", action="store_true", help="Allow commands to a real robot server")
    parser.add_argument("--no-gripper", action="store_true")
    parser.add_argument("--show-input", action="store_true")
    parser.add_argument("--record", action="store_true", help="Simulation only: record HDF5 episodes on the server")
    parser.add_argument("--task", default="")
    parser.add_argument("--seconds", type=float)
    args = parser.parse_args(argv)
    if args.arm == "both" and args.controller != "auto":
        parser.error("--controller left/right selects one hand; select --arm left or --arm right")
    if args.seconds is not None and (not np.isfinite(args.seconds) or args.seconds <= 0):
        parser.error("--seconds must be finite and positive")
    client = ControlClient(args.server)
    health = client.health()
    if args.demo and health["backend"] != "sim":
        parser.error("--demo is only allowed with a simulation server")
    names = ("left", "right") if args.arm == "both" else (args.arm,)
    if set(names) - set(health["arms"]):
        parser.error("The server does not have all selected arms")
    cfg = load_yaml(args.vr_config)
    policies = {}
    for name in names:
        settings = {**cfg.get("defaults", {}), **cfg.get("arms", {}).get(name, {})}
        if args.no_gripper:
            settings["gripper_enabled"] = False
        config = TeleopConfig(**settings)
        if len(names) == 1 and args.controller != "auto":
            config = bind_controller(config, args.controller[0])
        policies[name] = RelativeTeleop(config)
    source = None
    stop_reason = "operator"
    try:
        if not args.demo:
            source = AdbOculusSource(args.serial, args.adb)
            source.start(launch=not args.no_launch)
            auto = len(names) == 1 and args.controller == "auto"
            first_frame = source.wait_for_frame(
                ["l", "r"] if auto else [policy.config.controller for policy in policies.values()],
                progress=print if args.show_input else None,
                any_controller=auto,
            )
            if auto:
                name = names[0]
                config = policies[name].config
                controller = choose_controller(first_frame.poses, config.controller)
                policies[name] = RelativeTeleop(bind_controller(config, controller))
        bindings = {name: {"controller": policy.config.controller, "move_button": policy.config.move_button,
                           "gripper_button": policy.config.gripper_button,
                           "episode_button": policy.config.episode_button,
                           "discard_button": policy.config.discard_button} for name, policy in policies.items()}
        for name, binding in bindings.items():
            print(f"[Quest] robot={name}; controller={binding['controller']}; move={binding['move_button']}; "
                  f"gripper={binding['gripper_button']}; episode={binding['episode_button']}; "
                  f"discard={binding['discard_button']} (hold)")
        description = client.claim(allow_real=args.execute)
        metadata = {"source": "synthetic" if args.demo else "quest", "selected_arms": list(names),
                    "vr_config": cfg, "controller_bindings": bindings}
        if args.record:
            print(client.record(args.task, metadata))
        print("Side grip: hold to move. Front trigger: hold to close, release to open.")
        print("A/X: start an episode, or stop and save it. Hold B/Y: discard the latest episode.")
        print("Release controls before moving. Old or missing samples hold motion, then re-tare. Ctrl+C ends the session.")
        sequence = 0
        event_buttons = EventButtons(p.config for p in policies.values())
        hold_all = {name: {"hold_arm": True, "hold_gripper": True} for name in names}
        tracking_paused = None  # the reason motion is paused, or None
        event_was_held = False
        stale_after_hold = False  # policies skipped updates during a hold that sent no event
        last_control_at = None
        stale_timeout = min(p.config.stale_timeout_s for p in policies.values())
        started = last_print = time.monotonic()
        while args.seconds is None or time.monotonic() - started < args.seconds:
            before = time.monotonic()
            if source and source.error:
                raise ControllerInputError(source.error)
            obs = client.observation()
            sequence += 1
            if source:
                frame = source.latest()
                # Read the clock after the frame: the ADB thread can publish a
                # newer frame while latest() waits for its lock.
                now = time.monotonic()
            else:
                now = time.monotonic()
                frame = demo_frame(sequence, now, now - started)
            input_reason = input_pause_reason(frame, policies, now)
            loop_delayed = last_control_at is not None and now - last_control_at > stale_timeout
            last_control_at = now
            if input_reason:
                # Presses made while input is missing or old are ignored.
                event_buttons.reset()
                if input_reason != tracking_paused:
                    print(f"[Quest] {input_reason}; holding all arms and grippers.")
                tracking_paused = input_reason
                client.command(obs, hold_all, 0, frame.sequence if frame else -1, buttons=raw_buttons(frame))
                time.sleep(max(0, description["dt"] - (time.monotonic() - before)))
                continue
            events, event_held = event_buttons.update(frame, now)
            if events:
                # Events ride on a hold, so they share the command's ticket and
                # freshness checks. They are sent even while the robot side is
                # not executing: A/X is how an episode (and the agent) starts.
                result = client.command(obs, hold_all, max(0, time.monotonic() - frame.captured_at),
                                        frame.sequence, events=events, buttons=raw_buttons(frame))
                print(f"[Quest] sent {', '.join(events)}; server: {result.get('events')}")
                # Keep the selected physical hand, but discard all previous pose
                # origins and require a fresh grip/trigger release after the event.
                policies = {name: RelativeTeleop(p.config) for name, p in policies.items()}
                event_was_held, stale_after_hold = True, False
                time.sleep(max(0, description["dt"] - (time.monotonic() - before)))
                continue
            if not obs.get("enabled", True):
                # The robot side is not executing (agent inactive, node not ready).
                # Hold and re-tare when it resumes, so the arm cannot jump.
                reason = f"Robot side is not executing: {obs.get('status') or 'disabled'}"
                if reason != tracking_paused:
                    print(f"[Quest] {reason}; holding all arms and grippers.")
                tracking_paused = reason
                client.command(obs, hold_all, 0, frame.sequence, buttons=raw_buttons(frame))
                time.sleep(max(0, description["dt"] - (time.monotonic() - before)))
                continue
            if tracking_paused or loop_delayed:
                # Discard motion during the gap. A held side grip starts from
                # the current TCP; the gripper still needs release then press.
                policies = {name: RelativeTeleop(p.config, allow_held_grip=True) for name, p in policies.items()}
                tracking_paused = None
                print("[Quest] fresh input; motion re-tared. Release and press the trigger to use the gripper.")
            if event_held and not event_was_held:
                stale_after_hold = True
            if stale_after_hold and not event_held:
                # The policies were not updated while an event button was down.
                # Restart them; a held grip re-tares at the current TCP.
                policies = {name: RelativeTeleop(p.config, allow_held_grip=True) for name, p in policies.items()}
                stale_after_hold = False
            event_was_held = event_held
            actions = {}
            for name, policy in policies.items():
                if event_held:
                    actions[name] = {"hold_arm": True, "hold_gripper": True}
                    continue
                state = obs["arms"][name]
                command = policy.update(frame, state["tcp_pose"], state["gripper"], now)
                if command.fault:
                    raise ControllerInputError(f"{name}: {command.status}; stopping all arms and grippers")
                actions[name] = {"pose": command.pose, "gripper": command.gripper,
                                 "hold_arm": command.hold_arm, "hold_gripper": command.hold_gripper}
            # Only motion commands carry source age; holds do not use the pose.
            moving = any(a.get("pose") is not None or a.get("gripper") is not None for a in actions.values())
            age = max(0, time.monotonic() - frame.captured_at) if moving and frame else 0
            client.command(obs, actions, age, frame.sequence if frame else -1, buttons=raw_buttons(frame))
            if args.show_input and now - last_print >= 1:
                print({"source_age_s": time.monotonic() - frame.captured_at, "buttons": dict(frame.buttons),
                       "tcp": {name: state["tcp_pose"] for name, state in obs["arms"].items()}})
                last_print = now
            time.sleep(max(0, description["dt"] - (time.monotonic() - before)))
    except KeyboardInterrupt:
        pass
    except (RuntimeError, OSError) as exc:
        stop_reason = "controller input lost" if isinstance(exc, ControllerInputError) else "client error"
        parser.exit(1, f"xarm-vr: {exc}\n")
    finally:
        try:
            result = client.stop(reason=stop_reason)
            if result:
                print(result)
        except (OSError, RuntimeError) as exc:
            print(f"Stop request failed: {exc}. The server command timeout also holds motion.")
        try:
            client.close()
        finally:
            # End robot control before ADB cleanup, which can time out after USB loss.
            if source:
                try:
                    source.close(stop_app=True)
                except (OSError, RuntimeError) as exc:
                    print(f"Quest app cleanup failed: {exc}. The app may still be open.")


if __name__ == "__main__":
    main()
