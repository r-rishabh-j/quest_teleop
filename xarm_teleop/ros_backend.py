"""Backend for existing xarm_ros2 nodes, isolated from ROS's Python ABI."""
from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import threading
import time

import numpy as np


class RosBackend:
    kind = "ros"

    def __init__(self, names, python="/usr/bin/python3", no_gripper=False, mapping=None):
        self.names = tuple(names)
        self.dt = 0.02
        self.frame_id = 0
        self.done = dict.fromkeys(names, False)
        self.active = {name: [False, False] for name in names}
        self.targets = {}
        self._latest = None
        self._received = 0
        self._lock = threading.Lock()
        self._error = None
        mapping = mapping or {name: f"{name}_arm" for name in names}
        env = os.environ.copy()
        env["RCUTILS_LOGGING_USE_STDOUT"] = "0"
        command = [python, "-u", str(Path(__file__).with_name("ros_bridge.py")), "--mapping", json.dumps(mapping)]
        if no_gripper:
            command.append("--no-gripper")
        self._process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                         text=True, bufsize=1, env=env)
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()
        try:
            deadline = time.monotonic() + 12
            while time.monotonic() < deadline:
                if self._error:
                    raise RuntimeError(self._error)
                with self._lock:
                    ready = self._latest is not None and self._latest["ready"]
                if ready:
                    self.targets = {name: np.r_[state["tcp_pose"], state["gripper"]]
                                    for name, state in self.snapshot().items()}
                    return
                time.sleep(0.02)
            raise RuntimeError("Timed out waiting for the ROS bridge")
        except BaseException:
            self.close()
            raise

    def _read(self):
        try:
            for line in self._process.stdout:
                if not line.startswith("XARM_BRIDGE "):
                    continue
                message = json.loads(line[len("XARM_BRIDGE "):])
                with self._lock:
                    self._latest = message
                    self._received = time.monotonic()
        except Exception as exc:
            self._error = str(exc)
        finally:
            self._error = self._error or "ROS bridge exited; check its stderr output"

    def _send(self, message):
        if self._process.poll() is not None:
            raise RuntimeError("ROS bridge is not running")
        self._process.stdin.write(json.dumps(message, allow_nan=False) + "\n")
        self._process.stdin.flush()

    def describe(self):
        return {"backend": self.kind, "arms": list(self.names), "dt": self.dt, "rotation": True,
                "action": "absolute TCP pose: meters + rotvec radians; gripper: pulse [0,850]",
                "shared_bimanual_scene": True, "cameras": []}

    def snapshot(self):
        with self._lock:
            if self._error or not self._latest or not self._latest["ready"] or time.monotonic() - self._received > 0.3:
                raise RuntimeError(self._error or "ROS feedback is stale or control is stopped")
            return {name: dict(self._latest["arms"][name]) for name in self.names}

    def cameras(self):
        return {}

    def apply(self, commands):
        states = self.snapshot()
        actions = {}
        for name in self.names:
            command = commands.get(name)
            hold_arm = command is None or command.hold_arm
            hold_gripper = command is None or command.hold_gripper
            if hold_arm:
                self.targets[name][:6] = states[name]["tcp_pose"]
                self.active[name][0] = False
            if hold_gripper:
                self.targets[name][6] = states[name]["gripper"]
                self.active[name][1] = False
            if command and command.pose is not None:
                self.targets[name][:6] = command.pose
                self.active[name][0] = True
            if command and command.gripper is not None:
                self.targets[name][6] = command.gripper
                self.active[name][1] = True
            actions[name] = {"hold_arm": hold_arm, "hold_gripper": hold_gripper}
        # Send holds immediately, rather than waiting for the next server tick.
        self._send({"actions": actions})

    def tick(self):
        self.snapshot()
        actions = {name: {"pose": target[:6].tolist() if self.active[name][0] else None,
                          "gripper": float(target[6]) if self.active[name][1] else None,
                          "hold_arm": not self.active[name][0],
                          "hold_gripper": not self.active[name][1]}
                   for name, target in self.targets.items()}
        self._send({"actions": actions})
        self.frame_id += 1

    def actions(self):
        states = self.snapshot()
        # ROS feedback contains the last sent arm target. Gripper feedback is
        # measured position; the node does not expose a last-sent gripper target.
        applied = {name: np.r_[state["applied_pose"], state["gripper"]] for name, state in states.items()}
        return {name: target.copy() for name, target in self.targets.items()}, applied

    def hold(self):
        for name in self.names:
            self.active[name] = [False, False]
        self._send({"actions": {name: {"hold_arm": True, "hold_gripper": True} for name in self.names}})

    def reset(self, seed=0):
        raise ValueError("Automatic reset is disabled for physical robots")

    def close(self):
        try:
            if self._process.poll() is None:
                self._send({"stop": True})
                self._process.stdin.close()
                try:
                    self._process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self._process.terminate()
                    self._process.wait(timeout=2)
        finally:
            self._reader.join(timeout=1)
            self._process.stdout.close()
