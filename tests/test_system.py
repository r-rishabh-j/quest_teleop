import ast
import os
from pathlib import Path
import tempfile
import threading
import time
import types
import unittest
from unittest.mock import patch

import h5py
import numpy as np
from scipy.spatial.transform import Rotation

from xarm_teleop.backends import SimBackend
from xarm_teleop.client import ControlClient
from xarm_teleop.env import XArmLiftEnv
from xarm_teleop.ros_bridge import pose_vector
from xarm_teleop.server import ControlService
from xarm_teleop.transport import Connection, RemoteError, SocketServer, decode, encode
from xarm_teleop.teleop import bind_controller, choose_controller
from vr_support.core import TeleopConfig, RelativeTeleop, VRFrame
from vr_support.runtime import CONFIG_DIR


class ControllerBindingTests(unittest.TestCase):
    def test_either_single_controller_can_operate_the_selected_arm(self):
        config = TeleopConfig(translation_scale=0.2)
        for hand, grip, gripper in (("l", "LG", "LTr"), ("r", "RG", "RTr")):
            chosen = choose_controller([hand], config.controller)
            bound = bind_controller(config, chosen)
            self.assertEqual((bound.controller, bound.move_button, bound.gripper_button), (hand, grip, gripper))
            self.assertEqual(bound.translation_scale, 0.2)
            np.testing.assert_array_equal(bound.vr_to_robot, config.vr_to_robot)

    def test_custom_button_roles_follow_the_selected_hand(self):
        bound = bind_controller(TeleopConfig(move_button="RTr", gripper_button="B", episode_button="A"), "l")
        self.assertEqual((bound.move_button, bound.gripper_button), ("LTr", "Y"))
        self.assertEqual(bound.episode_button, "X")

    def test_two_available_controllers_keep_configured_preference(self):
        self.assertEqual(choose_controller(["l", "r"], "r"), "r")
        self.assertEqual(choose_controller(["l", "r"], "l"), "l")

    def test_losing_selected_hand_does_not_switch_to_other_hand(self):
        policy = RelativeTeleop(bind_controller(TeleopConfig(), choose_controller(["r"], "r")))
        robot = np.array([0.4, 0, 0.3, 0, 0, 0])
        policy.update(VRFrame(1, 1, {"r": np.eye(4)}, {"RG": 0, "RTr": 0}), robot, 850, 1)
        moving = policy.update(VRFrame(2, 1.02, {"r": np.eye(4)}, {"RG": 1, "RTr": 0}), robot, 850, 1.02)
        self.assertIsNotNone(moving.pose)
        lost = policy.update(VRFrame(3, 1.04, {"l": np.eye(4)}, {"LG": 1, "X": 0}), robot, 850, 1.04)
        self.assertTrue(lost.hold_arm)
        self.assertIsNone(lost.pose)


class EnvironmentTests(unittest.TestCase):
    def setUp(self):
        self.env = XArmLiftEnv()
        self.obs, _ = self.env.reset(seed=0)

    def tearDown(self):
        self.env.close()

    def test_gym_contract_and_deterministic_reset(self):
        self.assertTrue(self.env.observation_space.contains(self.obs))
        obs, _ = self.env.reset(seed=0)
        for name in obs:
            np.testing.assert_allclose(obs[name], self.obs[name])
        action = np.r_[obs["tcp_pose"], 1.0]
        result = self.env.step(action)
        self.assertEqual(len(result), 5)
        self.assertTrue(self.env.observation_space.contains(result[0]))

    def test_timestep_limits_and_rotation(self):
        initial = self.env.data.time
        old = self.env._target.copy()
        action = old.copy()
        action[0] += 0.5
        action[3:6] = (Rotation.from_rotvec([0, 0, 0.2]) * Rotation.from_rotvec(old[3:6])).as_rotvec()
        _, _, _, _, info = self.env.step(action)
        self.assertAlmostEqual(self.env.data.time - initial, self.env.dt)
        self.assertLessEqual(np.linalg.norm(info["applied_action"][:3] - old[:3]), self.env.max_vel * self.env.dt + 1e-10)
        angle = (Rotation.from_rotvec(info["applied_action"][3:6]) * Rotation.from_rotvec(old[3:6]).inv()).magnitude()
        self.assertGreater(angle, 0)
        self.assertLessEqual(angle, self.env.max_ang_vel * self.env.dt + 1e-10)

    def test_gripper_closes_and_reopens(self):
        action = np.r_[self.obs["tcp_pose"], 0.0]
        for _ in range(80):
            obs, *_ = self.env.step(action)
        self.assertLess(obs["gripper"][0], 0.1)
        action[-1] = 1
        for _ in range(80):
            obs, *_ = self.env.step(action)
        self.assertGreater(obs["gripper"][0], 0.9)

    def test_scripted_pick_can_lift_the_cube(self):
        action = np.r_[self.obs["object_pose"][:3], self.obs["tcp_pose"][3:], 1.0]
        action[2] += 0.04
        for _ in range(160):
            self.env.step(action)
        action[2] -= 0.04
        for _ in range(100):
            self.env.step(action)
        action[-1] = 0
        for _ in range(100):
            self.env.step(action)
        action[2] += 0.2
        for _ in range(160):
            _, _, terminated, _, _ = self.env.step(action)
        self.assertTrue(terminated)

    def test_invalid_action_does_not_mutate_physics(self):
        before = self.env.data.qpos.copy()
        with self.assertRaises(ValueError):
            self.env.step([float("nan")] * 7)
        np.testing.assert_array_equal(before, self.env.data.qpos)

    @unittest.skipUnless(os.environ.get("XARM_TEST_RENDER") == "1", "Set XARM_TEST_RENDER=1 in a graphics session")
    def test_rgb_render(self):
        pixels = self.env.render()
        self.assertEqual(pixels.shape, (256, 256, 3))
        self.assertGreater(int(pixels.max()) - int(pixels.min()), 20)


class ServiceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.backend = SimBackend(("left", "right"), images=False)
        self.service = ControlService(self.backend, self.temp.name)
        self.peer = object()
        self.lease = self.call("claim")["lease"]

    def tearDown(self):
        self.service.close()
        self.temp.cleanup()

    def call(self, op, data=None):
        request = {"op": op, "data": data or {}, "lease": getattr(self, "lease", None)}
        # Exercise serialization as well as dispatch without opening a robot socket.
        result = self.service.dispatch(self.peer, decode(bytearray(encode(request))))
        if not result["ok"]:
            raise ValueError(result["error"])
        return result["result"]

    def command(self, sequence=0, **overrides):
        obs = self.call("observation")
        target = list(obs["arms"]["right"]["tcp_pose"])
        target[0] += 0.03
        data = {"ticket": obs["ticket"], "sequence": sequence,
                "actions": {"right": {"pose": target, "gripper": 850}}}
        data.update(overrides)
        return self.call("command", data)

    def test_only_selected_arm_moves(self):
        before = self.backend.targets["left"].copy()
        self.command()
        self.service.tick()
        self.assertEqual(self.backend.active["left"], [False, False])
        self.assertEqual(self.backend.active["right"], [True, True])
        np.testing.assert_allclose(self.backend.targets["left"], before)

    def test_second_client_cannot_take_control(self):
        result = self.service.dispatch(object(), {"op": "claim"})
        self.assertFalse(result["ok"])
        self.assertEqual(self.service.lease, self.lease)

    def test_replay_revokes_control_and_holds(self):
        obs = self.call("observation")
        data = {"ticket": obs["ticket"], "sequence": 0, "actions": {}}
        self.call("command", data)
        with self.assertRaises(ValueError):
            self.call("command", data)
        self.assertIsNone(self.service.lease)
        self.assertTrue(all(flags == [False, False] for flags in self.backend.active.values()))

    def test_timeout_does_not_resume_old_motion(self):
        self.command()
        self.service.deadline = time.monotonic() - 1
        self.service.tick()
        self.assertIsNone(self.service.lease)
        self.assertFalse(self.backend.active["right"][0])
        with self.assertRaises(ValueError):
            self.call("observation")

    def test_timeout_reports_reason_and_stop_is_idempotent(self):
        self.command()
        self.service.deadline = time.monotonic() - 1
        self.service.tick()
        result = self.service.dispatch(self.peer, {"op": "observation", "lease": self.lease})
        self.assertFalse(result["ok"])
        self.assertEqual(result["reason"], "command timeout")
        self.assertNotIn("another connection", result["error"])
        with patch.object(self.backend, "hold") as hold:
            self.assertEqual(self.call("stop")["reason"], "command timeout")
            self.call("stop")
            hold.assert_not_called()

    def test_old_stop_cannot_stop_a_new_owner(self):
        self.call("stop")
        other = object()
        new_lease = self.service.dispatch(other, {"op": "claim"})["result"]["lease"]
        with patch.object(self.backend, "hold") as hold:
            self.call("stop")
            hold.assert_not_called()
        self.assertEqual(self.service.lease, new_lease)
        self.assertIs(self.service.owner, other)

    def test_interactive_sim_has_no_60_second_cutoff(self):
        for env in self.backend.envs.values():
            env._steps = 3000
        self.command()
        self.service.tick()
        self.assertEqual(self.service.lease, self.lease)
        self.assertFalse(any(self.backend.done.values()))

    def test_no_simulation_step_or_reset_between_sessions(self):
        self.call("stop")
        times = [env.data.time for env in self.backend.envs.values()]
        with patch.object(self.backend, "reset") as reset:
            for _ in range(5):
                self.service.tick()
            reset.assert_not_called()
        self.assertEqual(times, [env.data.time for env in self.backend.envs.values()])

    def test_explicit_step_limit_ends_without_starting_another_episode(self):
        for env in self.backend.envs.values():
            env.max_steps = 1
        with patch.object(self.backend, "reset") as reset:
            self.service.tick()
            self.service.tick()
            reset.assert_not_called()
        self.assertIsNone(self.service.lease)
        self.assertEqual(self.service.last_stop["reason"], "simulation time limit")

    def test_explicit_episode_reset_saves_then_resets_and_starts_recording(self):
        self.call("episode/start", {"task": "first"})
        self.command()
        self.service.tick()
        obs = self.call("observation")
        result = self.call("episode/new", {"ticket": obs["ticket"], "sequence": 1,
                                          "record": True, "task": "second"})
        self.assertNotEqual(result["lease"], self.lease)
        self.lease = result["lease"]
        self.assertEqual(self.backend.frame_id, 0)
        self.assertTrue(all(flags == [False, False] for flags in self.backend.active.values()))
        self.assertEqual(self.service.recorder.frames, 0)
        with h5py.File(result["previous_episode"]) as episode:
            self.assertEqual(episode.attrs["stop_reason"], "operator reset")
            self.assertEqual(episode.attrs["frames"], 1)

    def test_replayed_episode_reset_does_not_reset_again(self):
        obs = self.call("observation")
        payload = {"ticket": obs["ticket"], "sequence": 0}
        old_lease = self.lease
        result = self.call("episode/new", payload)
        with patch.object(self.backend, "reset") as reset:
            rejected = self.service.dispatch(self.peer, {"op": "episode/new", "lease": old_lease, "data": payload})
            self.assertFalse(rejected["ok"])
            reset.assert_not_called()
        self.assertEqual(self.service.lease, result["lease"])

    def test_physical_timeout_closes_backend_and_requires_manual_restart(self):
        # This is the physical-backend contract; no SDK/ROS connection is opened.
        physical = types.SimpleNamespace(kind="ros", done={"left": False, "right": False},
            snapshot=lambda: {}, hold=lambda: None, describe=lambda: {"backend": "ros"},
            close=unittest.mock.Mock(), tick=unittest.mock.Mock())
        service = ControlService(physical, self.temp.name)
        peer = object()
        lease = service.dispatch(peer, {"op": "claim"})["result"]["lease"]
        service.deadline = time.monotonic() - 1
        service.tick()
        service.tick()
        physical.close.assert_called_once()
        physical.tick.assert_not_called()
        self.assertFalse(service.dispatch(peer, {"op": "claim"})["ok"])
        self.assertEqual(service.dispatch(peer, {"op": "stop", "lease": lease})["result"]["reason"], "command timeout")
        service.close()
        physical.close.assert_called_once()

    def test_b_reset_never_resets_a_physical_backend(self):
        physical = types.SimpleNamespace(kind="ros", done={"right": False},
            snapshot=lambda: {}, hold=lambda: None, describe=lambda: {"backend": "ros"},
            close=unittest.mock.Mock(), reset=unittest.mock.Mock())
        service = ControlService(physical, self.temp.name)
        peer = object()
        lease = service.dispatch(peer, {"op": "claim"})["result"]["lease"]
        result = service.dispatch(peer, {"op": "episode/new", "lease": lease})
        self.assertFalse(result["ok"])
        self.assertIn("simulation-only", result["error"])
        physical.reset.assert_not_called()
        physical.close.assert_called_once()
        service.close()

    def test_real_client_updates_its_lease_after_explicit_episode_reset(self):
        self.call("stop")
        peer = object()
        def request(message):
            response = decode(bytearray(encode(self.service.dispatch(peer, message))))
            if not response["ok"]:
                raise RemoteError(response)
            return response["result"]
        client = ControlClient()
        client.connection = types.SimpleNamespace(request=request, close=lambda: None)
        client.claim()
        old = client.lease
        client.new_episode(client.observation())
        self.assertNotEqual(client.lease, old)
        self.assertEqual(client.lease, self.service.lease)
        self.assertEqual(client.sequence, 0)
        result = client.command(client.observation(), {})
        self.assertEqual(result["accepted"], 1)
        client.stop()

    def test_source_age_and_invalid_second_arm_fail_as_one_transaction(self):
        with self.assertRaises(ValueError):
            self.command(source_age_s=1.0)
        self.assertFalse(self.backend.active["right"][0])
        self.lease = None
        self.lease = self.call("claim")["lease"]
        obs = self.call("observation")
        with self.assertRaises(ValueError):
            self.call("command", {"ticket": obs["ticket"], "sequence": 1, "actions": {
                "right": {"pose": obs["arms"]["right"]["tcp_pose"]}, "left": {"pose": [1, 2]}}})
        self.assertFalse(self.backend.active["right"][0])

    def test_client_disconnect_finishes_episode_and_holds(self):
        self.call("episode/start", {"task": "disconnect test"})
        self.command()
        self.service.tick()
        self.service.disconnected(self.peer)
        self.assertIsNone(self.service.lease)
        with h5py.File(self.service.last_episode) as episode:
            self.assertEqual(episode.attrs["stop_reason"], "client disconnected")
            self.assertFalse(episode.attrs["success"])

    def test_recording_matches_before_action_and_after(self):
        self.call("episode/start", {"task": "pairing test"})
        before = self.backend.snapshot()
        self.command()
        self.service.tick()
        after = self.backend.snapshot()
        result = self.call("episode/stop")
        with h5py.File(result["episode"]) as episode:
            self.assertEqual(episode.attrs["frames"], 1)
            self.assertTrue(episode.attrs["complete"])
            np.testing.assert_allclose(episode["observation/right/state"][0, :6], before["right"]["tcp_pose"])
            np.testing.assert_allclose(episode["next_observation/right/state"][0, :6], after["right"]["tcp_pose"])
            self.assertEqual(episode["action/right/applied"].shape, (1, 7))
            self.assertEqual(episode["observation/right/state"].shape, (1, 14))

    def test_wrong_authentication_cannot_stop_owner(self):
        result = self.service.dispatch(self.peer, {"op": "stop", "lease": self.lease}, token="secret")
        self.assertFalse(result["ok"])
        self.assertEqual(self.service.lease, self.lease)

    def test_failed_record_write_does_not_leave_a_partial_transition(self):
        self.call("episode/start")
        recorder = self.service.recorder
        append = recorder._append
        def fail_midway(key, value):
            if key == "action/left/requested":
                raise OSError("simulated write failure")
            append(key, value)
        with patch.object(recorder, "_append", side_effect=fail_midway):
            with self.assertRaises(OSError):
                self.service.tick()
        self.assertEqual(recorder.frames, 0)
        lengths = []
        recorder.file.visititems(lambda name, item: lengths.append(len(item)) if isinstance(item, h5py.Dataset) else None)
        self.assertTrue(all(length == 0 for length in lengths))


class TransportTests(unittest.TestCase):
    def test_partial_frames_and_nonfinite_json(self):
        frame = encode({"x": [1, 2, 3]})
        for split in range(len(frame)):
            buffer = bytearray(frame[:split])
            self.assertIsNone(decode(buffer))
            buffer.extend(frame[split:])
            self.assertEqual(decode(buffer), {"x": [1, 2, 3]})
            self.assertFalse(buffer)
        with self.assertRaises(ValueError):
            decode(bytearray(b"\xff\xff\xff\xff"))
        with self.assertRaises(ValueError):
            encode({"x": float("nan")})

    def test_persistent_unix_connection(self):
        with tempfile.TemporaryDirectory() as folder:
            endpoint = "unix://" + folder + "/test.sock"
            closed = []
            try:
                server = SocketServer(endpoint, lambda p, m: {"ok": True, "result": m}, closed.append)
            except PermissionError:
                self.skipTest("This tool sandbox blocks listening sockets")
            stop = threading.Event()
            thread = threading.Thread(target=lambda: self._serve(server, stop), daemon=True)
            thread.start()
            try:
                connection = Connection(endpoint)
                for n in range(5):
                    self.assertEqual(connection.request({"sequence": n}), {"sequence": n})
                connection.close()
            finally:
                stop.set()
                thread.join(timeout=1)
                server.close()
            self.assertTrue(closed)

    @staticmethod
    def _serve(server, stop):
        while not stop.is_set():
            server.poll(0.01)

    def test_ros_backend_requires_explicit_execute(self):
        client = ControlClient()
        client.health = lambda: {"backend": "ros"}
        with self.assertRaises(ValueError):
            client.claim()

    def test_transport_timeout_discards_connection_and_lease(self):
        client = ControlClient()
        closed = []
        def timeout(_message):
            raise TimeoutError("late response")
        client.connection = types.SimpleNamespace(request=timeout, close=lambda: closed.append(True))
        client.lease = "old lease"
        with self.assertRaises(TimeoutError):
            client.observation()
        self.assertIsNone(client.connection)
        self.assertIsNone(client.lease)
        self.assertEqual(closed, [True])


class RosContractTests(unittest.TestCase):
    def test_ros_quaternion_and_joint_payload_are_distinct(self):
        pose = types.SimpleNamespace(position=types.SimpleNamespace(x=0.4, y=0, z=0.3),
            orientation=types.SimpleNamespace(x=0, y=1, z=0, w=0))
        np.testing.assert_allclose(pose_vector(pose), [0.4, 0, 0.3, 0, np.pi, 0])

    def test_actual_node_callbacks_accept_holds_and_reject_nan(self):
        # Load only these methods so tests exercise the real node code without
        # importing ROS binaries or opening a physical xArm connection.
        path = CONFIG_DIR.parents[1] / "xarm_ros2/xarm_scheduled_node.py"
        tree = ast.parse(path.read_text())
        node_class = next(n for n in tree.body if isinstance(n, ast.ClassDef))
        selected = [n for n in node_class.body if isinstance(n, ast.FunctionDef)
                    and n.name in ("command_callback", "gripper_command_callback")]
        module = ast.Module(body=selected, type_ignores=[])
        namespace = {"np": np}
        exec(compile(module, str(path), "exec"), namespace)
        calls = []
        ctrl = types.SimpleNamespace(hold_current_pose=lambda: calls.append("arm hold"),
                                     hold_current_position=lambda: calls.append("gripper hold"),
                                     command_pose=lambda *a: calls.append("arm move"),
                                     command_gripper_position=lambda *a: calls.append("gripper move"))
        node = types.SimpleNamespace(controller=ctrl, _gripper_ctrl=ctrl,
                                     get_logger=lambda: types.SimpleNamespace(warn=lambda text: None))
        for name in ("command_callback", "gripper_command_callback"):
            namespace[name](node, types.SimpleNamespace(cmd=[], duration=0.06))
            namespace[name](node, types.SimpleNamespace(cmd=[float("nan")] * (6 if name == "command_callback" else 1), duration=0.06))
        self.assertEqual(calls, ["arm hold", "gripper hold"])

    def test_start_refuses_a_tcp_outside_workspace(self):
        path = CONFIG_DIR.parents[1] / "xarm_ros2/xarm_scheduled_node.py"
        tree = ast.parse(path.read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "start_callback")
        namespace = {"np": np}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), "exec"), namespace)
        ctrl = types.SimpleNamespace(arm=types.SimpleNamespace(error_code=0),
                                    get_actual_tcp_pose=lambda: np.array([1, 0, 0.4, 0, 0, 0]),
                                    lower_bounds=np.array([0.3, -0.2, 0.3]),
                                    upper_bounds=np.array([0.5, 0.2, 0.5]))
        response = types.SimpleNamespace()
        namespace["start_callback"](types.SimpleNamespace(controller=ctrl), None, response)
        self.assertFalse(response.success)
        self.assertIn("workspace", response.message)


if __name__ == "__main__":
    unittest.main()
