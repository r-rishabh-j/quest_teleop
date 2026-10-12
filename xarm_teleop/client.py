"""Small ROS-free client usable by teleoperation, replay, or a learned policy."""
from __future__ import annotations

import argparse
import json
import os
import time

import numpy as np

from .transport import DEFAULT_ENDPOINT, Connection, RemoteError


class ControlClient:
    def __init__(self, endpoint=DEFAULT_ENDPOINT, token=None, timeout=2.0):
        self.endpoint = endpoint
        self.token = token if token is not None else os.environ.get("XARM_TELEOP_TOKEN")
        self.timeout = timeout
        self.lease = None
        self.sequence = 0
        self.connection = None

    def request(self, operation, payload=None):
        if self.connection is None:
            self.connection = Connection(self.endpoint, self.timeout)
        try:
            return self.connection.request({"op": operation.lstrip("/"), "data": payload or {},
                                            "lease": self.lease, "token": self.token})
        except RemoteError as exc:
            if exc.control_ended:
                self.lease = None
            raise
        except (OSError, ValueError):
            # After a timeout, a late reply could be mistaken for the next one.
            # Close the connection and let the server hold; never retry motion.
            self.close()
            raise

    def close(self):
        if self.connection:
            self.connection.close()
            self.connection = None
        self.lease = None

    def health(self):
        return self.request("/health")

    def claim(self, allow_real=False):
        health = self.health()
        if health["backend"] in ("real", "ros") and not allow_real:
            raise ValueError("Real robot control requires allow_real=True / --execute")
        result = self.request("/claim", {})
        self.lease = result["lease"]
        self.sequence = 0
        return result

    def observation(self, images=False):
        return self.request("/observation", {"images": images})

    def command(self, observation, actions, source_age_s=0.0, source_sequence=-1):
        self.sequence += 1
        return self.request("/command", {"ticket": observation["ticket"], "sequence": self.sequence,
            "actions": actions, "source_age_s": source_age_s, "source_sequence": source_sequence})

    def record(self, task="", metadata=None):
        return self.request("/episode/start", {"task": task, "metadata": metadata or {}})

    def stop_recording(self, success=False):
        result = self.request("/episode/stop", {"success": success})
        self.lease = None
        return result

    def new_episode(self, observation, task="", metadata=None, record=False, source_age_s=0.0, source_sequence=-1):
        self.sequence += 1
        result = self.request("/episode/new", {"ticket": observation["ticket"], "sequence": self.sequence,
            "task": task, "metadata": metadata or {}, "record": record,
            "source_age_s": source_age_s, "source_sequence": source_sequence})
        self.lease = result["lease"]
        self.sequence = 0
        return result

    def stop(self, success=False, reason="operator"):
        if self.lease is None:
            return None
        try:
            return self.request("/stop", {"success": success, "reason": reason})
        finally:
            self.lease = None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("health", "observe", "reset", "smoke"))
    parser.add_argument("--server", default=DEFAULT_ENDPOINT)
    parser.add_argument("--seconds", type=float, default=3)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--record", action="store_true")
    args = parser.parse_args(argv)
    if not np.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("--seconds must be finite and positive")
    client = ControlClient(args.server)
    if args.command == "health":
        print(json.dumps(client.health(), indent=2))
    elif args.command == "observe":
        print(json.dumps(client.observation(), indent=2))
    elif args.command == "reset":
        print(json.dumps(client.request("/reset", {"seed": args.seed}), indent=2))
    else:
        description = client.claim()  # This command refuses a physical backend.
        try:
            if args.record:
                print(client.record("Network smoke test (synthetic; not a human demonstration)"))
            first = client.observation()
            poses = {name: np.array(state["tcp_pose"]) for name, state in first["arms"].items()}
            start = time.monotonic()
            while time.monotonic() - start < args.seconds:
                before = time.monotonic()
                obs = client.observation()
                actions = {}
                for name, pose in poses.items():
                    target = pose.copy()
                    target[0] += 0.02 * np.sin((before - start) * 2)
                    actions[name] = {"pose": target.tolist(), "gripper": 850.0}
                client.command(obs, actions)
                time.sleep(max(0, description["dt"] - (time.monotonic() - before)))
            print("Network smoke test finished.")
        finally:
            print(client.stop())


if __name__ == "__main__":
    main()
