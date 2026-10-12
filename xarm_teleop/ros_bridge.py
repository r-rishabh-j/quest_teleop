"""ROS-side process. Run with the ROS system Python, not the uv interpreter.

Only the standard library and ROS packages are used here. JSON over stdin/stdout
keeps ROS Humble's Python 3.10 ABI out of the simulator's Python environment.
"""
import argparse
import json
import math
import sys
import threading
import time


def pose_vector(pose):
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    norm = math.sqrt(sum(v * v for v in q))
    if not math.isfinite(norm) or norm < 1e-10:
        raise ValueError("Invalid ROS TCP quaternion")
    q = [v / norm for v in q]
    if q[3] < 0:
        q = [-v for v in q]
    sine = math.sqrt(sum(v * v for v in q[:3]))
    scale = 2 * math.atan2(sine, q[3]) / sine if sine > 1e-10 else 2.0
    result = [pose.position.x, pose.position.y, pose.position.z] + [v * scale for v in q[:3]]
    if not all(math.isfinite(v) for v in result):
        raise ValueError("Non-finite ROS TCP pose")
    return result


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile
    from rclpy.duration import Duration
    from geometry_msgs.msg import PoseArray
    from std_msgs.msg import Float64, String
    from std_srvs.srv import Trigger
    from xarm_ctrl_msgs.msg import ScheduledCmd

    parser = argparse.ArgumentParser()
    parser.add_argument("--mapping", required=True)
    parser.add_argument("--no-gripper", action="store_true")
    args = parser.parse_args()
    mapping = json.loads(args.mapping)
    rclpy.init()
    node = Node("xarm_teleop_bridge")
    states, seen, grip_seen, statuses, status_seen = {}, {}, {}, {}, {}
    publishers, grip_publishers, starts, stops = {}, {}, {}, {}
    latest = [None]
    lock = threading.Lock()
    eof = threading.Event()
    # Depth one prevents old target queues. Lifespan further limits DDS backlog.
    command_qos = QoSProfile(depth=1, lifespan=Duration(seconds=0.15))

    def feedback(name, msg):
        if len(msg.poses) < 3:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        ros_now = node.get_clock().now().nanoseconds / 1e9
        if not -0.05 <= ros_now - stamp <= 0.3:
            return
        try:
            # This node packs seven joint angles into a Pose; its orientation
            # fields are joint values, not a quaternion.
            joints = msg.poses[2]
            q = [joints.position.x, joints.position.y, joints.position.z,
                 joints.orientation.x, joints.orientation.y, joints.orientation.z, joints.orientation.w]
            if not all(math.isfinite(v) for v in q):
                return
            previous = states.get(name, {})
            states[name] = {"tcp_pose": pose_vector(msg.poses[0]), "joint_positions": q,
                            "applied_pose": pose_vector(msg.poses[1]),
                            "gripper": previous.get("gripper", 0.0),
                            "timestamp": stamp, "sim_time": 0.0,
                            "reward": 0.0, "done": False, "success": False}
            seen[name] = time.monotonic()
        except ValueError:
            return

    def gripper_feedback(name, msg):
        if math.isfinite(msg.data):
            states.setdefault(name, {})["gripper"] = float(msg.data)
            grip_seen[name] = time.monotonic()

    def status_feedback(name, msg):
        try:
            statuses[name] = json.loads(msg.data)
            status_seen[name] = time.monotonic()
        except ValueError:
            pass

    for name, prefix in mapping.items():
        prefix = "/" + prefix.strip("/")
        publishers[name] = node.create_publisher(ScheduledCmd, prefix + "/xarm_command_pose", command_qos)
        grip_publishers[name] = node.create_publisher(ScheduledCmd, prefix + "/xarm_command_gripper", command_qos)
        node.create_subscription(PoseArray, prefix + "/xarm_state", lambda msg, n=name: feedback(n, msg), 1)
        node.create_subscription(Float64, prefix + "/xarm_gripper_state", lambda msg, n=name: gripper_feedback(n, msg), 1)
        node.create_subscription(String, prefix + "/xarm_control_status", lambda msg, n=name: status_feedback(n, msg), 1)
        starts[name] = node.create_client(Trigger, prefix + "/xarm_start_control")
        stops[name] = node.create_client(Trigger, prefix + "/xarm_stop_control")

    def publish(publisher, command):
        msg = ScheduledCmd()
        msg.cmd = command
        msg.duration = 0.06
        publisher.publish(msg)

    def hold():
        for name in mapping:
            publish(publishers[name], [])
            if not args.no_gripper:
                publish(grip_publishers[name], [])

    def read_stdin():
        try:
            for line in sys.stdin:
                if len(line) > 65536:
                    break
                message = json.loads(line)
                with lock:
                    latest[0] = (time.monotonic(), message)
        finally:
            eof.set()

    threading.Thread(target=read_stdin, daemon=True).start()
    futures = {}
    last_input = time.monotonic()
    started = time.monotonic()
    next_report = 0.0
    try:
        while rclpy.ok() and not eof.is_set():
            rclpy.spin_once(node, timeout_sec=0.005)
            now = time.monotonic()
            for name, client in starts.items():
                if name not in futures and client.service_is_ready():
                    futures[name] = client.call_async(Trigger.Request())
                if name in futures and futures[name].done():
                    response = futures[name].result()
                    if response is None or not response.success:
                        raise RuntimeError(f"Cannot start {name}: {getattr(response, 'message', 'service failed')}")
            ready = all(
                name in seen and now - seen[name] < 0.3
                and now - status_seen.get(name, 0) < 0.3
                and statuses.get(name, {}).get("protocol_version") == 1
                and 0 < statuses[name].get("command_timeout_s", 0) <= 0.5
                and statuses[name].get("arm_running") and not statuses[name].get("error")
                and (args.no_gripper or (statuses[name].get("gripper_running") and now - grip_seen.get(name, 0) < 0.3))
                for name in mapping
            )
            with lock:
                incoming, latest[0] = latest[0], None
            if incoming:
                received_at, message = incoming
                if message.get("stop"):
                    break
                if now - received_at > 0.15 or not ready:
                    hold()
                else:
                    last_input = now
                    for name, action in message.get("actions", {}).items():
                        if name not in publishers:
                            raise ValueError("Unknown ROS arm")
                        if action.get("hold_arm"):
                            publish(publishers[name], [])
                        elif action.get("pose") is not None:
                            publish(publishers[name], action["pose"])
                        if not args.no_gripper:
                            if action.get("hold_gripper"):
                                publish(grip_publishers[name], [])
                            elif action.get("gripper") is not None:
                                publish(grip_publishers[name], [action["gripper"]])
            if not ready or now - last_input > 0.25:
                hold()
            if now >= next_report:
                print("XARM_BRIDGE " + json.dumps({"ready": ready, "arms": states}), flush=True)
                next_report = now + 0.02
            if now - started > 10 and not ready:
                raise RuntimeError("ROS feedback/control status is unavailable; check nodes, services and the patched controller")
    finally:
        hold()
        # Give DDS a short interval to deliver the holds before destroying publishers.
        until = time.monotonic() + 0.1
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.01)
        # Ending the bridge ends the streams. A client disconnect while the
        # server stays up only sends holds, so another client can claim control.
        pending = [client.call_async(Trigger.Request()) for client in stops.values() if client.service_is_ready()]
        until = time.monotonic() + 0.75
        while time.monotonic() < until and any(not future.done() for future in pending):
            rclpy.spin_once(node, timeout_sec=0.01)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
