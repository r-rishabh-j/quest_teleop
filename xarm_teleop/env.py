"""Gymnasium xArm-7 lift task using gym-xarm assets and the current MuJoCo API."""
from __future__ import annotations

import os
from pathlib import Path
import sys

# Select a supported GL backend before importing MuJoCo. No GL context is created
# for state-only observations. Linux users can choose egl or osmesa explicitly.
if sys.platform == "darwin":
    os.environ.setdefault("MUJOCO_GL", "glfw")

import gymnasium as gym
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation


ASSETS = Path(__file__).parent / "assets" / "gym_xarm"


def limit_pose(current, desired, linear_step, angular_step, lower, upper):
    target = np.asarray(desired, dtype=float).copy()
    if target.shape != (6,) or not np.isfinite(target).all():
        raise ValueError("Expected six finite pose values")
    target[:3] = np.clip(target[:3], lower, upper)
    delta = target[:3] - current[:3]
    target[:3] = current[:3] + delta * min(1, linear_step / max(np.linalg.norm(delta), 1e-12))
    start = Rotation.from_rotvec(current[3:])
    delta = (Rotation.from_rotvec(target[3:]) * start.inv()).as_rotvec()
    delta *= min(1, angular_step / max(np.linalg.norm(delta), 1e-12))
    target[3:] = (Rotation.from_rotvec(delta) * start).as_rotvec()
    return target


class XArmLiftEnv(gym.Env):
    """Absolute base-frame TCP pose (m, rotvec rad) + gripper opening [0,1].

    The mocap weld drives the flange through MuJoCo constraints. This is a
    teleoperation/data-pipeline test, not a calibrated xArm servo dynamics model.
    """

    metadata = {"render_modes": ["rgb_array"], "render_fps": 50}

    def __init__(self, render_mode=None, pixels=False, width=256, height=256,
                 max_steps=3000, max_vel=0.15, max_ang_vel=0.8, terminate_on_success=True):
        if render_mode not in (None, "rgb_array"):
            raise ValueError("Use rgb_array and the server preview for rendering")
        self.render_mode = render_mode
        self.pixels = pixels
        self.width, self.height = int(width), int(height)
        self.max_steps = max_steps
        if max_steps is not None and (type(max_steps) is not int or max_steps <= 0):
            raise ValueError("max_steps must be a positive integer or None")
        self.terminate_on_success = terminate_on_success
        self.max_vel, self.max_ang_vel = max_vel, max_ang_vel
        self.model = mujoco.MjModel.from_xml_path(str(ASSETS / "push.xml"))
        self.data = mujoco.MjData(self.model)
        self.n_substeps = 10
        self.dt = self.n_substeps * self.model.opt.timestep
        self.lower = np.array([0.20, -0.35, -0.10])
        self.upper = np.array([0.65, 0.35, 0.50])
        self._joint_ids = np.array([self.model.joint(f"joint{i}").id for i in range(1, 8)])
        self._qpos_ids = self.model.jnt_qposadr[self._joint_ids]
        self._dof_ids = self.model.jnt_dofadr[self._joint_ids]
        self._site_id = self.model.site("grasp").id
        self._base_id = self.model.body("link0").id
        self._flange_id = self.model.body("link7").id
        self._object_id = self.model.body("object").id
        self._mocap_id = int(self.model.body("robot0:mocap2").mocapid[0])
        self._weld_id = next(i for i in range(self.model.neq)
                             if self.model.eq_type[i] == mujoco.mjtEq.mjEQ_WELD)
        # MuJoCo 3 weld layout: anchor xyz, relative xyz, relative wxyz, torque scale.
        self.model.eq_data[self._weld_id, :10] = [0, 0, 0, 0, 0, 0, 1, 0, 0, 0]
        self._finger_ids = np.array([self.model.joint(name).id for name in
                                    ("left_inner_knuckle_joint", "right_inner_knuckle_joint")])
        self._renderer = None
        self.model.vis.global_.offwidth = max(640, self.width)
        self.model.vis.global_.offheight = max(480, self.height)
        self.action_space = gym.spaces.Box(
            np.r_[self.lower, [-2 * np.pi] * 3, 0].astype(np.float64),
            np.r_[self.upper, [2 * np.pi] * 3, 1].astype(np.float64), dtype=np.float64)
        spaces = {
            "tcp_pose": gym.spaces.Box(-np.inf, np.inf, (6,), np.float64),
            "joint_positions": gym.spaces.Box(-np.inf, np.inf, (7,), np.float64),
            "gripper": gym.spaces.Box(0, 1, (1,), np.float64),
            "object_pose": gym.spaces.Box(-np.inf, np.inf, (7,), np.float64),
        }
        if pixels:
            spaces["pixels"] = gym.spaces.Box(0, 255, (self.height, self.width, 3), np.uint8)
        self.observation_space = gym.spaces.Dict(spaces)
        self._reset_qpos = None
        self._steps = 0
        self._target = None

    def _tcp_pose(self):
        base_r = self.data.xmat[self._base_id].reshape(3, 3)
        pos = base_r.T @ (self.data.site_xpos[self._site_id] - self.data.xpos[self._base_id])
        rot = base_r.T @ self.data.site_xmat[self._site_id].reshape(3, 3)
        return np.r_[pos, Rotation.from_matrix(rot).as_rotvec()]

    def _opening(self):
        angles = self.data.qpos[self.model.jnt_qposadr[self._finger_ids]]
        return float(np.clip(1 - np.mean(angles) / 0.85, 0, 1))

    def _drive(self, pose, opening):
        base_r = self.data.xmat[self._base_id].reshape(3, 3)
        tcp_world = self.data.xpos[self._base_id] + base_r @ pose[:3]
        site_local_r = Rotation.from_quat(self.model.site_quat[self._site_id], scalar_first=True).as_matrix()
        flange_r = base_r @ Rotation.from_rotvec(pose[3:]).as_matrix() @ site_local_r.T
        self.data.mocap_pos[self._mocap_id] = tcp_world - flange_r @ self.model.site_pos[self._site_id]
        self.data.mocap_quat[self._mocap_id] = Rotation.from_matrix(flange_r).as_quat(scalar_first=True)
        target_angle = (1 - opening) * 0.85
        angles = self.data.qpos[self.model.jnt_qposadr[self._finger_ids]]
        velocities = self.data.qvel[self.model.jnt_dofadr[self._finger_ids]]
        # Convert a joint torque PD command into the model's motor control units.
        self.data.ctrl[:] = np.clip((40 * (target_angle - angles) - 2 * velocities) /
                                    self.model.actuator_gear[:, 0], -1, 1)

    def _initial_ik(self):
        target_pos = self.data.xpos[self._base_id] + [0.4, 0, 0.18]
        target_rot = Rotation.from_rotvec([np.pi, 0, 0])
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        for _ in range(400):
            mujoco.mj_forward(self.model, self.data)
            error = np.r_[target_pos - self.data.site_xpos[self._site_id],
                          (target_rot * Rotation.from_matrix(self.data.site_xmat[self._site_id].reshape(3, 3)).inv()).as_rotvec()]
            if np.linalg.norm(error) < 1e-5:
                return
            mujoco.mj_jacSite(self.model, self.data, jacp, jacr, self._site_id)
            jac = np.vstack((jacp, jacr))[:, self._dof_ids]
            dq = jac.T @ np.linalg.solve(jac @ jac.T + 1e-4 * np.eye(6), error)
            self.data.qpos[self._qpos_ids] += np.clip(dq, -0.1, 0.1)
            bounds = self.model.jnt_range[self._joint_ids]
            self.data.qpos[self._qpos_ids] = np.clip(self.data.qpos[self._qpos_ids], bounds[:, 0], bounds[:, 1])
        raise RuntimeError("Could not initialize the simulated arm with IK")

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        if self._reset_qpos is None:
            self._initial_ik()
            self._reset_qpos = self.data.qpos.copy()
        else:
            self.data.qpos[:] = self._reset_qpos
        object_qpos = self.data.joint("object_joint0").qpos
        object_qpos[:] = [1.50 + self.np_random.uniform(-0.04, 0.04),
                          0.28 + self.np_random.uniform(-0.08, 0.08), 0.567, 1, 0, 0, 0]
        mujoco.mj_forward(self.model, self.data)
        self._target = np.r_[self._tcp_pose(), 1.0]
        for _ in range(100):
            self._drive(self._target[:6], 1)
            mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self._target = np.r_[self._tcp_pose(), self._opening()]
        self._initial_object_z = float(self.data.xpos[self._object_id, 2])
        self._steps = 0
        return self._observation(), self._info()

    def _observation(self):
        pos = self.data.xpos[self._object_id] - self.data.xpos[self._base_id]
        obs = {"tcp_pose": self._tcp_pose(), "joint_positions": self.data.qpos[self._qpos_ids].copy(),
               "gripper": np.array([self._opening()]),
               "object_pose": np.r_[pos, self.data.xquat[self._object_id][[1, 2, 3, 0]]]}
        if self.pixels:
            obs["pixels"] = self.render()
        return obs

    def _info(self):
        return {"sim_time": float(self.data.time),
                "is_success": bool(self.data.xpos[self._object_id, 2] > self._initial_object_z + 0.12),
                "applied_action": self._target.copy()}

    def step(self, action):
        action = np.asarray(action, dtype=float)
        if action.shape != (7,) or not np.isfinite(action).all():
            raise ValueError("Action must contain a finite TCP pose and gripper opening")
        if self._target is None:
            raise RuntimeError("Call reset before step")
        self._target[:6] = limit_pose(self._target[:6], action[:6], self.max_vel * self.dt,
                                      self.max_ang_vel * self.dt, self.lower, self.upper)
        self._target[6] += np.clip(np.clip(action[6], 0, 1) - self._target[6], -self.dt, self.dt)
        for _ in range(self.n_substeps):
            self._drive(self._target[:6], self._target[6])
            mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self._steps += 1
        if not np.isfinite(self.data.qpos).all():
            raise RuntimeError("MuJoCo produced a non-finite state")
        info = self._info()
        distance = np.linalg.norm(self.data.site_xpos[self._site_id] - self.data.xpos[self._object_id])
        height = max(0, self.data.xpos[self._object_id, 2] - self._initial_object_z)
        reward = float(-distance + 5 * height + float(info["is_success"]))
        terminated = bool(self.terminate_on_success and info["is_success"])
        truncated = self.max_steps is not None and self._steps >= self.max_steps
        return self._observation(), reward, terminated, truncated, info

    def hold(self, arm=True, gripper=True):
        if arm:
            self._target[:6] = self._tcp_pose()
        if gripper:
            self._target[6] = self._opening()
        return self._target.copy()

    def render(self):
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height=self.height, width=self.width)
        self._renderer.update_scene(self.data, camera="camera0")
        return self._renderer.render().copy()

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None


gym.register("XArmTeleopLift-v0", entry_point="xarm_teleop.env:XArmLiftEnv")
