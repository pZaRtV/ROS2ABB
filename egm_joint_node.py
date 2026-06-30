#!/usr/bin/env python3
"""
Model-agnostic ABB EGM joint stream node for ROS2 Humble.
Receives EGM protobuf packets over UDP, publishes JointState,
and sends back joint reference commands.

No URDF or robot support package required.

Usage:
  1. Copy egm_pb2.py (compiled from egm.proto) to same directory
  2. ros2 run <pkg> egm_joint_node   OR   python3 egm_joint_node.py

RAPID side: EGMRunJoint must be active on the controller.
"""

import socket
import threading
import sys
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# ── Protobuf import ──────────────────────────────────────────────────────────
# Try to import from workspace install, then fall back to local directory
def _find_egm_pb2():
    search_roots = [
        '/home/roscom/ros2abb_ws/script',
        os.path.dirname(__file__),
    ]
    for root in search_roots:
        for dirpath, _, filenames in os.walk(root):
            if 'egm_pb2.py' in filenames:
                sys.path.insert(0, dirpath)
                return True
    return False

if not _find_egm_pb2():
    raise ImportError(
        "egm_pb2.py not found. Compile egm.proto with:\n"
        "  protoc --python_out=. egm.proto\n"
        "and place egm_pb2.py in the same directory as this script."
    )

import egm_pb2  # noqa: E402  (after path fixup)
# ─────────────────────────────────────────────────────────────────────────────


class EGMJointNode(Node):
    """
    Bridges ABB EGM UDP stream ↔ ROS2 topics.

    Subscribed topics:
      /egm/joint_command  [std_msgs/Float64MultiArray]
          Joint position reference in DEGREES (matches ABB convention).
          If not published, the node echoes current position (hold in place).

    Published topics:
      /egm/joint_states   [sensor_msgs/JointState]
          Live joint feedback from the robot in DEGREES.
    """

    def __init__(self):
        super().__init__('egm_joint_node')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('egm_port',   6511)
        self.declare_parameter('bind_ip',    '0.0.0.0')
        self.declare_parameter('num_joints', 6)
        self.declare_parameter('palletizer_mode', True)  # True for 4-axis IRB460
        self.declare_parameter('joint_names', [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6',
        ])

        self.port             = self.get_parameter('egm_port').value
        self.bind_ip          = self.get_parameter('bind_ip').value
        self.num_joints       = self.get_parameter('num_joints').value
        self.palletizer_mode  = self.get_parameter('palletizer_mode').value
        self.jnames           = self.get_parameter('joint_names').value[:self.num_joints]

        # ── State ─────────────────────────────────────────────────────────
        self._current_joints: list[float] = [0.0] * self.num_joints
        self._ref_joints:     list[float] | None = None   # None = echo back
        self._lock = threading.Lock()
        self._seq  = 0
        self._controller_addr = None

        # ── ROS2 interfaces ───────────────────────────────────────────────
        self._pub_js = self.create_publisher(JointState, '/egm/joint_states', 10)
        self.create_subscription(
            Float64MultiArray, '/egm/joint_command',
            self._cmd_cb, 10
        )

        # ── UDP socket ────────────────────────────────────────────────────
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.bind_ip, self.port))
        self._sock.settimeout(1.0)

        mode_str = "palletizer (J1,J2,J3,J6)" if self.palletizer_mode else "standard (J1-J6)"
        self.get_logger().info(
            f"EGM node ready — listening on UDP {self.bind_ip}:{self.port} "
            f"| joints: {self.num_joints} | mode: {mode_str}"
        )

        # ── UDP receive thread ────────────────────────────────────────────
        self._running = True
        self._thread  = threading.Thread(target=self._udp_loop, daemon=True)
        self._thread.start()

    # ── Command subscriber ────────────────────────────────────────────────
    def _cmd_cb(self, msg: Float64MultiArray):
        with self._lock:
            self._ref_joints = list(msg.data[:self.num_joints])

    # ── UDP receive/send loop ─────────────────────────────────────────────
    def _udp_loop(self):
        robot_msg  = egm_pb2.EgmRobot()
        sensor_msg = egm_pb2.EgmSensor()

        while self._running and rclpy.ok():
            try:
                data, addr = self._sock.recvfrom(4096)
            except socket.timeout:
                continue

            self._controller_addr = addr

            # ── Parse feedback ────────────────────────────────────────
            robot_msg.ParseFromString(data)
            all_joints = list(robot_msg.feedBack.joints.joints)  # always 6 elements

            # ── Extract physical joint feedback ────────────────────────
            # IRB460 palletizer: physical joints are at proto indices 0,1,2,5 (J1,J2,J3,J6)
            # Standard 6-axis:  physical joints are at proto indices 0,1,2,3,4,5
            if self.palletizer_mode:
                fb = [all_joints[0], all_joints[1], all_joints[2], all_joints[5]]
            else:
                fb = all_joints[:self.num_joints]

            with self._lock:
                self._current_joints = fb
                ref = (self._ref_joints or fb)[:]   # echo if no command

            # ── Publish JointState ────────────────────────────────────
            if rclpy.ok():
                js = JointState()
                js.header.stamp = self.get_clock().now().to_msg()
                js.name         = self.jnames
                js.position     = [j * 3.14159 / 180.0 for j in fb]  # deg→rad
                self._pub_js.publish(js)

            # ── Send reference back to controller ─────────────────────
            # IRB460 palletizer: user joints [j1,j2,j3,j4] → proto [j1,j2,j3,0,0,j4]
            # Standard 6-axis:  user joints [j1..j6]       → proto [j1,j2,j3,j4,j5,j6]
            if self.palletizer_mode:
                ref_padded = [ref[0], ref[1], ref[2], 0.0, 0.0, ref[3]]
            else:
                ref_padded = list(ref) + [0.0] * (6 - len(ref))

            sensor_msg.Clear()
            sensor_msg.header.seqno = self._seq
            sensor_msg.header.tm    = robot_msg.header.tm
            sensor_msg.header.mtype = 1  # MSGTYPE_CORRECTION: PC→controller sensor data
            sensor_msg.planned.joints.joints[:] = ref_padded

            self._sock.sendto(sensor_msg.SerializeToString(), addr)
            self._seq += 1

            if self._seq % 250 == 0 and self._ref_joints is not None:
                self.get_logger().info(f"Streaming ref: {ref_padded}")

    def destroy_node(self):
        self._running = False
        if self._thread.is_alive():
            self._thread.join(timeout=1.5)
        try:
            self._sock.close()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = EGMJointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
