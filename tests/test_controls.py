"""Exercise the real client loop with controller frames, without robot hardware."""
from contextlib import ExitStack, redirect_stderr, redirect_stdout
from copy import deepcopy
from dataclasses import replace
import io
import selectors
import types
import unittest
from unittest.mock import Mock, patch

import numpy as np

from vr_support.core import RelativeTeleop, TeleopConfig, VRFrame
from vr_support.oculus import LogDecoder
from xarm_teleop import teleop
from xarm_teleop.client import ControlClient
from xarm_teleop.transport import Peer, RemoteError, SocketServer, encode

POSE = [0.4, 0.0, 0.3, 0.0, 3.14, 0.0]


def vr_frame(sequence, grip=0, trigger=0, episode=0, hands=("r",), x=0):
    poses = {hand: np.eye(4) for hand in hands}
    for pose in poses.values():
        pose[0, 3] = x
    return VRFrame(sequence, 100.0, poses, {"RG": grip, "RTr": trigger, "B": episode,
                                          "LG": 0, "LTr": 0, "Y": 0})


class FakeClient:
    def __init__(self, names):
        self.names = names
        self.commands, self.resets, self.stops = [], [], []
        self.pose = list(POSE)
        self.lease = None

    def health(self):
        return {"backend": "sim", "arms": self.names, "dt": 0.02}

    def claim(self, **kwargs):
        self.lease = "test"
        return self.health()

    def observation(self):
        return {"ticket": "test", "arms": {name: {"tcp_pose": self.pose, "gripper": 850} for name in self.names}}

    def command(self, obs, actions, *args):
        self.commands.append(deepcopy(actions))

    def new_episode(self, *args, **kwargs):
        self.resets.append(len(self.commands))
        self.pose = list(POSE)
        return {"episode": None}

    def stop(self, **kwargs):
        self.stops.append(kwargs)
        self.lease = None
        return {"stopped": True}

    def close(self):
        pass


class FakeSource:
    error = None

    def __init__(self, frames):
        self.first = frames[0]
        self.frames = iter(frames)

    def start(self, **kwargs):
        pass

    def wait_for_frame(self, *args, **kwargs):
        return self.first

    def latest(self):
        try:
            return next(self.frames)
        except StopIteration:
            raise KeyboardInterrupt

    def close(self, **kwargs):
        pass


class TeleopControlsTests(unittest.TestCase):
    def run_frames(self, frames, arm="right", expect_failure=False, client=None, source=None, clock=None):
        client = client or FakeClient(["left", "right"] if arm == "both" else [arm])
        source = source or Mock(wraps=FakeSource(frames))
        source.error = None
        # Operator gain tuning must not change the geometry expected by these tests.
        config = {"defaults": {"translation_scale": 0.5}, "arms": {
            "left": {"controller": "l", "move_button": "LG", "gripper_button": "LTr", "episode_button": "Y"},
            "right": {"controller": "r", "move_button": "RG", "gripper_button": "RTr", "episode_button": "B"},
        }}
        with ExitStack() as stack:
            stack.enter_context(patch.object(teleop, "load_yaml", return_value=config))
            stack.enter_context(patch.object(teleop, "ControlClient", return_value=client))
            stack.enter_context(patch.object(teleop, "AdbOculusSource", return_value=source))
            stack.enter_context(patch.object(teleop.time, "monotonic", clock or Mock(return_value=100.0)))
            stack.enter_context(patch.object(teleop.time, "sleep"))
            stack.enter_context(redirect_stdout(io.StringIO()))
            stack.enter_context(redirect_stderr(io.StringIO()))
            if expect_failure:
                with self.assertRaises(SystemExit) as error:
                    teleop.main(["--arm", arm, "--no-launch"])
                self.assertEqual(error.exception.code, 1)
            else:
                teleop.main(["--arm", arm, "--no-launch"])
        source.close.assert_called_once_with(stop_app=True)
        return client

    def run_timed_frames(self, frames, times=None, **kwargs):
        clock = Mock(return_value=100.0)
        source = Mock(wraps=FakeSource(frames))
        frames_iter = iter(frames)
        times_iter = iter(times) if times is not None else None

        def latest():
            try:
                frame = next(frames_iter)
            except StopIteration:
                raise KeyboardInterrupt
            # Simulate a publication while latest() acquires the reader lock.
            clock.return_value = next(times_iter) if times_iter is not None else frame.captured_at + 0.001
            return frame

        source.latest.side_effect = latest
        return self.run_frames(frames, source=source, clock=clock, **kwargs)

    def test_frame_published_during_read_is_not_rejected_as_future_input(self):
        client = self.run_timed_frames([replace(vr_frame(1), captured_at=100.02)])
        self.assertEqual(len(client.commands), 1)
        self.assertEqual(client.stops, [{"reason": "operator"}])

    def test_robot_stop_precedes_app_stop_even_if_server_stop_fails(self):
        for stop_error in (None, RuntimeError("server disconnected")):
            with self.subTest(stop_error=stop_error):
                events = []
                client = FakeClient(["right"])
                client.stop = Mock(side_effect=lambda **kwargs: events.append("robot stop"))
                if stop_error:
                    def fail_stop(**kwargs):
                        events.append("robot stop")
                        raise stop_error
                    client.stop.side_effect = fail_stop
                client.close = lambda: events.append("connection close")
                source = Mock(wraps=FakeSource([vr_frame(1)]))
                source.close.side_effect = lambda **kwargs: events.append("app stop")
                self.run_frames([vr_frame(1)], client=client, source=source)
                self.assertEqual(events, ["robot stop", "connection close", "app stop"])

    def test_startup_error_stops_app_and_cleanup_error_does_not_mask_failure(self):
        source = Mock(wraps=FakeSource([vr_frame(1)]))
        source.start.side_effect = RuntimeError("startup failed")
        source.close.side_effect = RuntimeError("USB disconnected")
        self.run_frames([vr_frame(1)], source=source, expect_failure=True)
        source.wait_for_frame.assert_not_called()

    def test_front_trigger_closes_on_press_and_opens_on_release(self):
        client = self.run_frames([vr_frame(1), vr_frame(2, trigger=1), vr_frame(3, trigger=0)])
        self.assertIsNone(client.commands[0]["right"]["gripper"])
        self.assertEqual(client.commands[1]["right"]["gripper"], 0)
        self.assertEqual(client.commands[2]["right"]["gripper"], 850)

    def test_b_resets_once_and_requires_grip_and_trigger_release(self):
        client = self.run_frames([
            vr_frame(1), vr_frame(2, grip=1),
            vr_frame(3, grip=1, trigger=1, episode=1),
            vr_frame(4, grip=1, trigger=1, episode=1),
            vr_frame(5, grip=1, trigger=1), vr_frame(6),
            vr_frame(7, grip=1, trigger=1),
        ])
        self.assertEqual(len(client.resets), 1)
        held = client.commands[2]["right"]
        self.assertTrue(held["hold_arm"] and held["hold_gripper"])
        self.assertIsNone(client.commands[3]["right"]["pose"])
        self.assertIsNone(client.commands[3]["right"]["gripper"])
        np.testing.assert_allclose(client.commands[5]["right"]["pose"], POSE)
        self.assertEqual(client.commands[5]["right"]["gripper"], 0)

    def test_b_already_held_at_start_does_not_reset(self):
        client = self.run_frames([vr_frame(1, episode=1), vr_frame(2, episode=1), vr_frame(3)])
        self.assertEqual(client.resets, [])

    def test_losing_one_of_two_controllers_holds_both_without_reset(self):
        client = self.run_frames([vr_frame(1, hands=("l", "r")), vr_frame(2, hands=("l",))],
                                 arm="both")
        self.assertEqual(client.resets, [])
        self.assertEqual(len(client.commands), 2)
        for action in client.commands[1].values():
            self.assertEqual(action, {"hold_arm": True, "hold_gripper": True})

    def test_recovery_tares_held_grip_and_requires_new_gripper_press(self):
        frames = [
            (0.00, vr_frame(1)), (0.02, vr_frame(2, grip=1, trigger=1)),
            (0.04, vr_frame(3, hands=())),
            # Controller moves one metre during the gap. No robot catch-up.
            (0.06, vr_frame(4, grip=1, trigger=1, episode=1, x=1)),
            (0.16, vr_frame(5, grip=1, trigger=1, episode=1, x=1)),
            (0.27, vr_frame(6, grip=1, trigger=1, episode=1, x=1)),
            (0.29, vr_frame(7, grip=1, trigger=1, x=1)),
            (0.31, vr_frame(8, grip=1, x=1.02)),
            (0.33, vr_frame(9, grip=1, trigger=1, x=1.02)),
            (0.35, vr_frame(10, grip=1, x=1.02)),
        ]
        client = self.run_timed_frames([replace(f, captured_at=100 + t) for t, f in frames])
        self.assertEqual(client.resets, [])  # B during loss must not start an episode.
        for command in client.commands[2:5]:
            self.assertEqual(command["right"], {"hold_arm": True, "hold_gripper": True})
        # B was held at recovery, so both outputs stay held until B is released.
        self.assertTrue(client.commands[5]["right"]["hold_arm"])
        np.testing.assert_allclose(client.commands[6]["right"]["pose"], POSE)
        for command in client.commands[6:8]:
            self.assertIsNone(command["right"]["gripper"])
        self.assertEqual(client.commands[8]["right"]["gripper"], 0)
        self.assertEqual(client.commands[9]["right"]["gripper"], 850)
        self.assertAlmostEqual(client.commands[7]["right"]["pose"][1], POSE[1] - 0.01)

    def test_clutch_moves_on_next_sample_without_a_recovery_timer(self):
        # Reproduce two idle hands -> missing right pose -> right grip held.
        frames = [
            (0.00, vr_frame(1, hands=("l", "r"))),
            (0.02, vr_frame(2, hands=("l",))),
            (0.04, vr_frame(3, grip=1, x=1)),
            (0.06, vr_frame(4, grip=1, x=1.02)),
        ]
        client = self.run_timed_frames([replace(f, captured_at=100 + t) for t, f in frames])
        self.assertEqual(client.commands[1]["right"], {"hold_arm": True, "hold_gripper": True})
        np.testing.assert_allclose(client.commands[2]["right"]["pose"], POSE)
        self.assertAlmostEqual(client.commands[3]["right"]["pose"][1], POSE[1] - 0.01)
        self.assertEqual(client.resets, [])

    def test_auto_binding_never_switches_to_other_hand_during_loss(self):
        for selected, other in (("r", "l"), ("l", "r")):
            with self.subTest(selected=selected):
                frames = [replace(vr_frame(1, hands=(selected,)), captured_at=100)]
                frames += [replace(vr_frame(i + 2, hands=(other,)), captured_at=100 + (i + 1) * 0.1)
                           for i in range(20)]
                client = self.run_timed_frames(frames)
                self.assertEqual(client.resets, [])
                for command in client.commands[1:]:
                    self.assertEqual(command, {"right": {"hold_arm": True, "hold_gripper": True}})

    def test_two_arm_recovery_tares_both_and_preserves_each_hand_mapping(self):
        frames = []
        for seq, offset, hands, grip, x in (
            (1, 0.00, ("l", "r"), 0, 0), (2, 0.02, ("l", "r"), 1, 0),
            (3, 0.04, ("l",), 1, 0), (4, 0.06, ("l", "r"), 1, 1),
            (5, 0.27, ("l", "r"), 1, 1), (6, 0.29, ("l", "r"), 1, 1),
        ):
            f = replace(vr_frame(seq, grip=grip, hands=hands, x=x), captured_at=100 + offset)
            f.buttons["LG"] = grip
            frames.append(f)
        frames[-1].poses["r"][0, 3] += 0.04
        client = self.run_timed_frames(frames, arm="both")
        for action in client.commands[2].values():
            self.assertEqual(action, {"hold_arm": True, "hold_gripper": True})
        for arm in ("left", "right"):
            np.testing.assert_allclose(client.commands[3][arm]["pose"], POSE)
        np.testing.assert_allclose(client.commands[5]["left"]["pose"], POSE)
        self.assertAlmostEqual(client.commands[5]["right"]["pose"][1], POSE[1] - 0.02)
        self.assertEqual(client.resets, [])

    def test_stale_sample_holds_then_retares_without_ending_episode(self):
        for arm in ("right", "both"):
            with self.subTest(arm=arm):
                frames = [
                    vr_frame(1, hands=("l", "r")),
                    replace(vr_frame(2, grip=1, trigger=1, hands=("l", "r")), captured_at=100.02),
                    # A late B press must not reset the environment.
                    replace(vr_frame(3, grip=1, episode=1, hands=("l", "r")), captured_at=100.02),
                    None,
                    replace(vr_frame(4, grip=1, trigger=1, hands=("l", "r"), x=1), captured_at=100.62),
                    replace(vr_frame(5, grip=1, trigger=1, hands=("l", "r"), x=1.02), captured_at=100.64),
                ]
                client = self.run_timed_frames(frames, arm=arm,
                    times=[100.001, 100.021, 100.275535, 100.6, 100.621, 100.641])
                self.assertEqual(len(client.commands), len(frames))
                for command in client.commands[2:4]:
                    for action in command.values():
                        self.assertEqual(action, {"hold_arm": True, "hold_gripper": True})
                np.testing.assert_allclose(client.commands[4]["right"]["pose"], POSE)
                self.assertIsNone(client.commands[4]["right"]["gripper"])
                self.assertAlmostEqual(client.commands[5]["right"]["pose"][1], POSE[1] - 0.01)
                self.assertEqual(client.resets, [])
                self.assertEqual(client.stops, [{"reason": "operator"}])  # Test ends with Ctrl+C.

    def test_delayed_control_loop_with_fresh_input_retares(self):
        frames = [vr_frame(1), replace(vr_frame(2, grip=1), captured_at=100.02),
                  replace(vr_frame(3, grip=1, x=1), captured_at=100.29),
                  replace(vr_frame(4, grip=1, x=1.02), captured_at=100.31)]
        client = self.run_timed_frames(frames)
        np.testing.assert_allclose(client.commands[2]["right"]["pose"], POSE)
        self.assertAlmostEqual(client.commands[3]["right"]["pose"][1], POSE[1] - 0.01)
        self.assertEqual(client.resets, [])
        self.assertEqual(client.stops, [{"reason": "operator"}])

    def test_pose_jump_stops_session_without_reset(self):
        client = self.run_frames([vr_frame(1), vr_frame(2, grip=1), vr_frame(3, grip=1, x=1)], expect_failure=True)
        self.assertEqual(client.resets, [])
        self.assertEqual(len(client.commands), 2)
        self.assertEqual(client.stops, [{"reason": "controller input lost"}])


class InputValidationTests(unittest.TestCase):
    def test_sample_age_diagnostics_distinguish_stale_and_future_frames(self):
        policies = {"right": RelativeTeleop(TeleopConfig())}
        reason = teleop.input_pause_reason(replace(vr_frame(1), captured_at=99.7), policies, 100)
        self.assertIn("age=0.300000s", reason)
        with self.assertRaisesRegex(teleop.ControllerInputError, "age=-0.010000s"):
            teleop.input_pause_reason(replace(vr_frame(1), captured_at=100.01), policies, 100)

    def test_bad_packet_is_not_reported_as_tracking_loss(self):
        decoder = LogDecoder(0, started_at=0)
        frame = decoder.decode("100.000000 123 456 I wE9ryARX: r:bad&R,RG", 100)
        policies = {"right": RelativeTeleop(TeleopConfig())}
        with self.assertRaisesRegex(teleop.ControllerInputError, "Invalid Quest packet"):
            teleop.input_pause_reason(frame, policies, 100)
        self.assertTrue(policies["right"].update(frame, POSE, 850, 100).fault)


class ReplyHandlingTests(unittest.TestCase):
    def test_server_ended_session_does_not_send_second_stop_request(self):
        client = ControlClient()
        connection = Mock()
        connection.request.side_effect = RemoteError({"error": "Control ended: command timeout",
                                                       "control_ended": True, "reason": "command timeout"})
        client.connection, client.lease = connection, "old lease"
        with self.assertRaises(RemoteError):
            client.observation()
        self.assertIsNone(client.lease)
        self.assertIsNone(client.stop())
        self.assertEqual(connection.request.call_count, 1)

    def test_reply_is_sent_during_read_poll_and_backpressure_keeps_write_interest(self):
        for blocked in (False, True):
            with self.subTest(blocked=blocked):
                sock = Mock()
                sock.recv.return_value = encode({"op": "health"})
                sock.send.side_effect = BlockingIOError() if blocked else lambda value: len(value)
                peer = Peer(sock)
                server = SocketServer.__new__(SocketServer)
                server.listener = object()
                server.selector = Mock()
                server.selector.select.return_value = [(types.SimpleNamespace(fileobj=sock, data=peer), selectors.EVENT_READ)]
                server.selector.get_map.return_value = {}
                server.dispatch = lambda peer, message: {"ok": True, "result": {}}
                server.disconnected = Mock()
                server.poll(0)
                sock.send.assert_called_once()
                self.assertEqual(bool(peer.outgoing), blocked)
                expected = selectors.EVENT_READ | (selectors.EVENT_WRITE if blocked else 0)
                server.selector.modify.assert_called_with(sock, expected, peer)


if __name__ == "__main__":
    unittest.main()
