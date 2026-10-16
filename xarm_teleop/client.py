"""xarm-client: health, observation, reset and smoke-test commands for a server.

ControlClient lives in vr_support.link, shared with manipulation_env's QuestAgent.
"""
from __future__ import annotations

import argparse
import json
import time

import numpy as np

from vr_support.link import DEFAULT_ENDPOINT, ControlClient

__all__ = ["ControlClient", "main"]


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
