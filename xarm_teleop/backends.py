"""The server is the sole owner of simulation or robot connections."""
from __future__ import annotations

import time

import numpy as np

from vr_support.core import TeleopCommand
from vr_support.runtime import LiveRobot, load_yaml, make_settings


def state_vector(state):
    return np.r_[state["tcp_pose"], state["joint_positions"], state["gripper"]]


class SimBackend:
    kind = "sim"

    def __init__(self, names, images=True, seed=0, max_steps=None, stop_on_success=False):
        from .env import XArmLiftEnv
        self.names = tuple(names)
        self.envs = {}
        self.targets = {}
        self.active = {name: [False, False] for name in names}
        self.observations = {}
        self.info = {}
        self.rewards = dict.fromkeys(names, 0.0)
        self.done = dict.fromkeys(names, False)
        self.frame_id = 0
        self.images = images
        self.max_steps, self.stop_on_success = max_steps, stop_on_success
        self.terminated = dict.fromkeys(names, False)
        self.truncated = dict.fromkeys(names, False)
        try:
            for i, name in enumerate(names):
                env = XArmLiftEnv(pixels=images, max_steps=max_steps, terminate_on_success=stop_on_success)
                self.envs[name] = env
                self.observations[name], self.info[name] = env.reset(seed=seed + i)
                self.targets[name] = np.r_[self.observations[name]["tcp_pose"], self.observations[name]["gripper"]]
        except BaseException:
            self.close()
            raise
        self.dt = next(iter(self.envs.values())).dt

    def describe(self):
        return {"backend": self.kind, "arms": list(self.names), "dt": self.dt,
                "action": "absolute TCP pose: meters + rotvec radians; gripper: pulse [0,850]",
                "rotation": True, "shared_bimanual_scene": False,
                "episode_steps": self.max_steps, "stop_on_success": self.stop_on_success,
                "cameras": [f"{name}/rgb" for name in self.names] if self.images else []}

    def snapshot(self):
        return {name: {"tcp_pose": obs["tcp_pose"].copy(),
                       "joint_positions": obs["joint_positions"].copy(),
                       "gripper": float(obs["gripper"][0] * 850),
                       "object_pose": obs["object_pose"].copy(),
                       "timestamp": time.time(), "sim_time": self.info[name]["sim_time"],
                       "reward": self.rewards[name], "done": self.done[name],
                       "terminated": self.terminated[name], "truncated": self.truncated[name],
                       "success": self.info[name]["is_success"]}
                for name, obs in self.observations.items()}

    def cameras(self):
        return {f"{name}/rgb": obs["pixels"] for name, obs in self.observations.items() if "pixels" in obs}

    def apply(self, commands):
        for name in self.names:
            command = commands.get(name, TeleopCommand(hold_arm=True, hold_gripper=True))
            if command.hold_arm or command.hold_gripper:
                held = self.envs[name].hold(command.hold_arm, command.hold_gripper)
                if command.hold_arm:
                    self.targets[name][:6] = held[:6]
                    self.active[name][0] = False
                if command.hold_gripper:
                    self.targets[name][6] = held[6]
                    self.active[name][1] = False
            if command.pose is not None:
                self.targets[name][:6] = command.pose
                self.active[name][0] = True
            if command.gripper is not None:
                self.targets[name][6] = command.gripper / 850
                self.active[name][1] = True

    def tick(self):
        for name, env in self.envs.items():
            if self.done[name]:
                continue
            obs, reward, terminated, truncated, info = env.step(self.targets[name])
            self.observations[name], self.info[name] = obs, info
            self.rewards[name] = reward
            self.terminated[name], self.truncated[name] = bool(terminated), bool(truncated)
            self.done[name] = bool(terminated or truncated)
        self.frame_id += 1

    def actions(self):
        requested = {name: np.r_[target[:6], target[6] * 850] for name, target in self.targets.items()}
        applied = {name: np.r_[info["applied_action"][:6], info["applied_action"][6] * 850]
                   for name, info in self.info.items()}
        return requested, applied

    def hold(self):
        self.apply({})

    def reset(self, seed=0):
        for i, (name, env) in enumerate(self.envs.items()):
            self.observations[name], self.info[name] = env.reset(seed=seed + i)
            self.targets[name] = np.r_[self.observations[name]["tcp_pose"], self.observations[name]["gripper"]]
            self.active[name] = [False, False]
            self.rewards[name], self.done[name] = 0.0, False
            self.terminated[name], self.truncated[name] = False, False
        self.frame_id = 0

    def close(self):
        for env in self.envs.values():
            env.close()


class RealBackend:
    kind = "real"

    def __init__(self, names, robot_config, vr_config, no_gripper=False):
        self.names = tuple(names)
        self.control, settings = make_settings(load_yaml(robot_config), load_yaml(vr_config), names,
                                              no_gripper=no_gripper)
        self.dt = 1 / self.control["rate_hz"]
        self.robots = {}
        self.targets = {}
        self.active = {name: [False, False] for name in names}
        self.frame_id = 0
        self.done = dict.fromkeys(names, False)
        try:
            for name, (policy, kwargs, pose) in settings.items():
                self.robots[name] = LiveRobot(policy, kwargs, pose, self.control)
                actual, grip = self.robots[name].state()
                self.targets[name] = np.r_[actual, grip if grip is not None else 0.0]
        except BaseException:
            self.close()
            raise

    def describe(self):
        return {"backend": self.kind, "arms": list(self.names), "dt": self.dt,
                "action": "absolute TCP pose: meters + rotvec radians; gripper: pulse [0,850]",
                "rotation": True, "shared_bimanual_scene": True, "cameras": []}

    def snapshot(self):
        result = {}
        for name, robot in self.robots.items():
            pose, grip = robot.state()
            joints = robot.arm.get_actual_joint_positions()
            if joints is None or len(joints) != 7 or not np.isfinite(joints).all():
                raise RuntimeError("xArm-7 joint feedback is unavailable")
            result[name] = {"tcp_pose": pose, "joint_positions": joints,
                            "gripper": float(grip if grip is not None else 0),
                            "timestamp": time.time(), "sim_time": 0.0,
                            "reward": 0.0, "done": False, "success": False}
        return result

    def cameras(self):
        return {}  # Add a timestamped camera source here for physical RGB recording.

    def apply(self, commands):
        for name, robot in self.robots.items():
            command = commands.get(name, TeleopCommand(hold_arm=True, hold_gripper=True))
            if command.hold_arm:
                robot.arm.hold_current_pose()
                self.targets[name][:6] = robot.arm.get_current_target_pose()
                self.active[name][0] = False
            if command.hold_gripper:
                if robot.gripper:
                    robot.gripper.hold_current_position()
                    self.targets[name][6] = robot.gripper.get_current_target_gripper()
                self.active[name][1] = False
            if command.pose is not None:
                self.targets[name][:6] = command.pose
                self.active[name][0] = True
            if command.gripper is not None:
                self.targets[name][6] = command.gripper
                self.active[name][1] = True

    def tick(self):
        for name, robot in self.robots.items():
            robot.state()  # Check faults even while held.
            robot.apply(TeleopCommand(pose=self.targets[name][:6] if self.active[name][0] else None,
                                      gripper=self.targets[name][6] if self.active[name][1] else None),
                        self.control["duration_s"])
        self.frame_id += 1

    def actions(self):
        applied = {}
        for name, robot in self.robots.items():
            pose = robot.arm.get_current_target_pose()
            grip = robot.gripper.get_current_target_gripper() if robot.gripper else 0.0
            applied[name] = np.r_[pose, grip]
        return {name: value.copy() for name, value in self.targets.items()}, applied

    def hold(self):
        self.apply({})

    def reset(self, seed=0):
        raise ValueError("Automatic reset is disabled for real robots")

    def close(self):
        for name, robot in reversed(list(self.robots.items())):
            try:
                robot.close()
            except Exception as exc:
                print(f"[{name}] cleanup failed: {exc}")
