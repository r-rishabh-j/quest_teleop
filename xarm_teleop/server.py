"""Single-owner socket control server. Simulation and GL run on the server thread."""
from __future__ import annotations

import argparse
import base64
from contextlib import nullcontext
import hmac
import io
import os
from pathlib import Path
import secrets
import signal
import time

import numpy as np
from PIL import Image

from vr_support.link import (DEFAULT_ENDPOINT, PROTOCOL_VERSION, SocketServer, is_loopback,
                             parse_actions, parse_buttons, parse_events)
from vr_support.runtime import CONFIG_DIR
from .backends import RealBackend, SimBackend
from .recording import EpisodeRecorder
from .ros_backend import RosBackend


class ControlService:
    def __init__(self, backend, directory="episodes", timeout=0.3, reset_seed=0):
        self.backend = backend
        self.recorder = EpisodeRecorder(directory)
        self.timeout = timeout
        self.lease = None
        self.deadline = 0.0
        self.ticket = None
        self.ticket_time = 0.0
        self.sequence = -1
        self.source_sequence = -1
        self.state = backend.snapshot()
        self.state_at = time.monotonic()
        self.last_episode = None
        self.owner = None
        self.last_stop = None
        self._ended_lease = None
        self._ended_owner = None
        self.backend_stopped = False
        self.reset_seed = reset_seed
        self.record_args = None  # (task, metadata) while the client records

    def _is_ended_session(self, peer, lease):
        return lease is not None and lease == self._ended_lease and peer is self._ended_owner

    def _ended_reply(self):
        reason = self.last_stop["reason"]
        if self.backend_stopped:
            recovery = "Restart the server and client explicitly; release controls before moving."
        elif reason in ("task complete", "simulation time limit"):
            recovery = "Reset the simulation, then restart the client."
        else:
            recovery = "Release the controls, then restart the client."
        return {"ok": False, "code": "control_ended", "control_ended": True,
                "reason": reason, "episode": self.last_stop["episode"],
                "error": f"Control ended: {reason}. All selected arms and grippers are stopped. {recovery}"}

    def dispatch(self, peer, message, token=None):
        if token and (not isinstance(message.get("token"), str) or not hmac.compare_digest(message["token"], token)):
            return {"ok": False, "error": "Authentication required"}
        lease = message.get("lease")
        try:
            op, payload = message["op"], message.get("data", {})
            if not isinstance(payload, dict):
                raise ValueError("Request data must be an object")
            # Ending a session clears its owner. Check the closed session first,
            # otherwise a normal timeout looks like an attempt by a second client.
            if self._is_ended_session(peer, lease):
                if op == "stop":
                    return {"ok": True, "result": dict(self.last_stop)}
                return self._ended_reply()
            if lease:
                if lease != self.lease:
                    raise ValueError("Control session is invalid or expired")
                if peer is not self.owner:
                    raise ValueError("Lease belongs to another connection")
                self.require_lease(lease)
            if op == "health":
                result = self.describe()
            elif op == "claim":
                result = self.claim()
                self.owner = peer
            elif op == "observation":
                result = self.observation(lease)
                if payload.get("images"):
                    result["images"] = {}
                    for name, pixels in self.backend.cameras().items():
                        buffer = io.BytesIO()
                        Image.fromarray(pixels).save(buffer, format="JPEG", quality=80)
                        result["images"][name] = {"encoding": "jpeg", "data": base64.b64encode(buffer.getvalue()).decode()}
            elif op == "command":
                result = self.command(lease, payload)
            elif op == "stop":
                self.require_lease(lease)
                if type(payload.get("success", False)) is not bool:
                    raise ValueError("success must be boolean")
                reason = payload.get("reason", "operator")
                if reason not in ("operator", "controller input lost", "client error"):
                    raise ValueError("Invalid stop reason")
                result = self.stop(reason, success=payload.get("success", False))
            elif op == "reset":
                if self.lease:
                    raise ValueError("Stop the control client before reset")
                self.backend.reset(seed=int(payload.get("seed", 0)))
                self.state = self.backend.snapshot()
                self.state_at = time.monotonic()
                result = self.observation()
            elif op == "episode/start":
                self.require_lease(lease)
                self.record_args = (str(payload.get("task", "")), payload.get("metadata"))
                result = {"episode": self.recorder.start(self.describe(), *self.record_args)}
            elif op == "episode/stop":
                self.require_lease(lease)
                if type(payload.get("success", False)) is not bool:
                    raise ValueError("success must be boolean")
                result = self.stop("episode stopped", success=payload.get("success", False))
            else:
                raise ValueError("Unknown operation")
            return {"ok": True, "result": result}
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            if self.lease and lease == self.lease and peer is self.owner:
                self.stop("invalid request")
            ended = self._is_ended_session(peer, lease)
            if ended and self.last_stop["reason"] == "command timeout":
                return self._ended_reply()
            return {"ok": False, "error": str(exc), "code": "request_rejected",
                    "control_ended": ended,
                    "reason": self.last_stop["reason"] if ended else None}

    def disconnected(self, peer):
        if peer is self.owner:
            self.stop("client disconnected")
            self.owner = None

    def describe(self):
        return {**self.backend.describe(), "protocol_version": PROTOCOL_VERSION,
                "command_timeout_s": self.timeout, "recording": self.recorder.file is not None,
                "control_claimed": self.lease is not None, "last_episode": self.last_episode,
                "last_stop": self.last_stop, "backend_stopped": self.backend_stopped}

    def claim(self):
        if self.backend_stopped:
            raise ValueError("Physical robot streams are stopped. Restart the server explicitly before a new session")
        if self.lease is not None:
            raise ValueError("Another client owns control")
        if any(self.backend.done.values()):
            raise ValueError("Episode is done; reset the simulation before claiming control")
        self.backend.hold()
        self.state = self.backend.snapshot()
        self.state_at = time.monotonic()
        self.lease = secrets.token_urlsafe(24)
        self.deadline = time.monotonic() + self.timeout
        self.sequence = -1
        self.ticket = None
        return {"lease": self.lease, **self.describe()}

    def require_lease(self, lease):
        if not self.lease or lease != self.lease:
            raise ValueError("Control lease is invalid or expired; reconnect and release the buttons")
        if time.monotonic() > self.deadline:
            self.stop("command timeout")
            raise ValueError("Control lease expired; reconnect and release the buttons")

    def observation(self, lease=None):
        # The simulator executes every accepted command, so it is always enabled.
        result = {"frame_id": self.backend.frame_id, "arms": self.state, "enabled": True, "status": ""}
        if lease:
            self.require_lease(lease)
            if time.monotonic() - self.state_at > self.timeout:
                self.stop("observation timeout")
                raise ValueError("Server observation is too old")
            self.ticket = secrets.token_urlsafe(16)
            self.ticket_time = time.monotonic()
            result["ticket"] = self.ticket
        return result

    def command(self, lease, payload):
        self.require_lease(lease)
        try:
            if payload.get("ticket") != self.ticket or self.ticket is None:
                raise ValueError("Missing, replayed, or superseded observation ticket")
            self.ticket = None
            sequence = payload["sequence"]
            if type(sequence) is not int or sequence <= self.sequence:
                raise ValueError("Command sequence must increase")
            source_sequence = payload.get("source_sequence", -1)
            if type(source_sequence) is not int:
                raise ValueError("source_sequence must be an integer")
            source_age = float(payload.get("source_age_s", 0))
            if not np.isfinite(source_age) or source_age < 0 or (
                time.monotonic() - self.ticket_time + source_age > self.timeout
            ):
                raise ValueError("Command input or observation is too old")
            commands = parse_actions(payload["actions"], self.backend.names)
            events = parse_events(payload.get("events", []))
            parse_buttons(payload.get("buttons", {}))  # validated only; the simulator does not log them
            if "episode" in events and self.backend.kind != "sim":
                raise ValueError("Episode reset is simulation-only; reset physical robots manually")
            self.backend.apply(commands)
            self.sequence = sequence
            self.source_sequence = source_sequence
            self.deadline = time.monotonic() + self.timeout
        except (ValueError, KeyError, TypeError, OverflowError):
            self.stop("invalid command")
            raise
        # Events ride on a command, so a delayed or replayed press is rejected
        # by the same ticket and sequence checks as motion.
        results = {name: self.episode_event() if name == "episode" else "ignored" for name in events}
        return {"accepted": sequence, "frame_id": self.backend.frame_id, "events": results}

    def episode_event(self):
        """B/Y: finish the recording, reset the scene, and keep the same control session."""
        previous = self.recorder.stop(all(s["success"] for s in self.state.values()), "operator reset")
        if previous:
            self.last_episode = previous
        self.backend.reset(seed=self.reset_seed)
        self.backend.hold()
        self.state = self.backend.snapshot()
        self.state_at = time.monotonic()
        episode = None
        if self.record_args is not None:
            episode = self.recorder.start(self.describe(), *self.record_args)
        return {"reset": True, "previous_episode": previous, "episode": episode}

    def tick(self):
        if self.backend_stopped:
            return
        if self.lease is not None and time.monotonic() > self.deadline:
            self.stop("command timeout")
            return
        # Waiting for the Quest must not consume a simulation episode's step
        # budget. Physical backends still tick to check feedback and faults.
        if self.backend.kind == "sim" and self.lease is None:
            self.state = self.backend.snapshot()
            self.state_at = time.monotonic()
            return
        before = self.state
        cameras = self.backend.cameras()
        timestamp = time.time()
        self.backend.tick()
        self.state = self.backend.snapshot()
        self.state_at = time.monotonic()
        requested, applied = self.backend.actions()
        self.recorder.append(before, self.state, requested, applied, self.backend.active,
                             cameras, timestamp, self.source_sequence)
        if self.lease is not None and any(self.backend.done.values()):
            success = all(s["success"] for s in self.state.values())
            reason = ("simulation time limit" if any(s.get("truncated") for s in self.state.values()) and not success
                      else "task complete")
            self.stop(reason, success=success)

    def stop(self, reason="operator", success=False):
        ended_lease, ended_owner = self.lease, self.owner
        self.lease = self.ticket = None
        self.owner = None
        self.record_args = None
        try:
            if self.backend.kind == "sim":
                self.backend.hold()
            elif not self.backend_stopped:
                # A lost client must stop the physical arm and gripper streams,
                # not just leave servo threads waiting for another client.
                self.backend_stopped = True
                self.backend.close()
        finally:
            path = self.recorder.stop(success, reason)
            if path:
                self.last_episode = path
            if ended_lease is not None:
                self._ended_lease, self._ended_owner = ended_lease, ended_owner
                self.last_stop = {"stopped": True, "reason": reason, "episode": path}
                print(f"[control] ended: {reason}", flush=True)
        return self.last_stop or {"stopped": True, "reason": reason, "episode": self.last_episode}

    def close(self):
        try:
            self.stop("server shutdown")
        finally:
            if not self.backend_stopped:
                self.backend.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("sim", "real", "ros"), default="sim")
    parser.add_argument("--arm", choices=("left", "right", "both"), default="right")
    parser.add_argument("--endpoint", default=DEFAULT_ENDPOINT)
    parser.add_argument("--viewer", action="store_true", help="Open a MuJoCo window (use mjpython on macOS)")
    parser.add_argument("--view-arm", choices=("left", "right"), help="Scene shown when controlling two simulated arms")
    parser.add_argument("--no-images", action="store_true")
    parser.add_argument("--no-gripper", action="store_true")
    parser.add_argument("--execute", action="store_true", help="Required for the real robot backend")
    parser.add_argument("--robot-config", type=Path, default=CONFIG_DIR / "roamlab.yaml")
    parser.add_argument("--vr-config", type=Path, default=CONFIG_DIR / "quest3.yaml")
    parser.add_argument("--episodes", type=Path, default=Path("episodes"))
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--episode-steps", type=int, help="Optional simulation step limit; interactive teleop has no limit by default")
    parser.add_argument("--stop-on-success", action="store_true", help="End simulation control when the lift task succeeds")
    parser.add_argument("--ros-python", default="/usr/bin/python3", help="Python that can import the sourced ROS installation")
    parser.add_argument("--ros-left-name", default="left_arm")
    parser.add_argument("--ros-right-name", default="right_arm")
    parser.add_argument("--seconds", type=float, help="Optional run duration for smoke tests")
    args = parser.parse_args(argv)
    token = os.environ.get("XARM_TELEOP_TOKEN")
    if not is_loopback(args.endpoint) and not token:
        parser.error("Set XARM_TELEOP_TOKEN before binding to a network interface")
    if args.backend in ("real", "ros") and not args.execute:
        parser.error("A physical robot backend requires --execute")
    if args.viewer and args.backend != "sim":
        parser.error("--viewer requires --backend sim")
    if args.seconds is not None and (not np.isfinite(args.seconds) or args.seconds <= 0):
        parser.error("--seconds must be finite and positive")
    if args.episode_steps is not None and args.episode_steps <= 0:
        parser.error("--episode-steps must be positive")
    if args.backend != "sim" and (args.episode_steps is not None or args.stop_on_success):
        parser.error("Episode step/success limits apply only to --backend sim")
    names = ("left", "right") if args.arm == "both" else (args.arm,)
    if args.view_arm and args.view_arm not in names:
        parser.error("--view-arm must be one of the selected arms")
    if args.backend == "sim":
        backend = SimBackend(names, not args.no_images, args.seed,
                             max_steps=args.episode_steps, stop_on_success=args.stop_on_success)
    elif args.backend == "ros":
        mapping = {name: getattr(args, f"ros_{name}_name") for name in names}
        backend = RosBackend(names, args.ros_python, args.no_gripper, mapping)
    else:
        backend = RealBackend(names, args.robot_config, args.vr_config, args.no_gripper)
    try:
        service = ControlService(backend, args.episodes, reset_seed=args.seed)
    except BaseException:
        backend.close()
        raise
    server = None
    viewer = None
    stopped = False

    def stop(*_):
        nonlocal stopped
        stopped = True

    previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        server = SocketServer(args.endpoint, lambda peer, msg: service.dispatch(peer, msg, token), service.disconnected)
        if args.viewer:
            import mujoco.viewer
            env = backend.envs[args.view_arm or names[0]]
            viewer = mujoco.viewer.launch_passive(env.model, env.data)
        print(f"{args.backend} server: {args.endpoint} arms={','.join(names)}", flush=True)
        print("Control is idle until a client claims it. Ctrl+C stops the server.", flush=True)
        started = next_tick = time.monotonic()
        while not stopped and (args.seconds is None or time.monotonic() - started < args.seconds):
            now = time.monotonic()
            if now >= next_tick:
                with viewer.lock() if viewer else nullcontext():
                    service.tick()
                if viewer:
                    if not viewer.is_running():
                        stopped = True
                    else:
                        viewer.sync()
                next_tick = max(next_tick + backend.dt, time.monotonic())
            server.poll(max(0.001, min(0.01, next_tick - time.monotonic())))
    finally:
        try:
            if server:
                server.close()
        finally:
            try:
                if viewer:
                    viewer.close()
            finally:
                service.close()
                for sig, handler in previous.items():
                    signal.signal(sig, handler)


if __name__ == "__main__":
    main()
