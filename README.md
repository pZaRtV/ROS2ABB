# ABB EGM + ROS2 Humble Setup Guide
## IRB460 Virtual Controller → Ubuntu VM (VMware)

> **Environment**
> - Host OS: Windows (RobotStudio 2022+)
> - Guest OS: Ubuntu 22.04 (VMware Workstation)
> - ROS2 Distribution: Humble
> - Robot: ABB IRB460 110/2.4 (virtual controller)
> - Driver: PickNik Robotics `abb_ros2`

---

## Architecture Overview

```
┌─────────────────────────────────┐      ┌──────────────────────────────────┐
│  Windows Host (RobotStudio)     │      │  Ubuntu VM (ROS2 Humble)         │
│                                 │      │                                  │
│  Virtual Controller             │      │  egm_joint_node.py               │
│  ├── RAPID: EGM_ROS2.mod        │      │  ├── UDP socket :6511            │
│  ├── EGMRunJoint (250 Hz)       │      │  ├── Publishes /egm/joint_states │
│  └── UCdevice (UDPUC)           │      │  └── Subscribes /egm/joint_cmd   │
│                                 │      │                                  │
│  VMnet1: 192.168.255.1  ◄──────UDP:6511──────►  ens37: 192.168.255.128   │
│                                 │      │                                  │
│  RWS (TCP:80) ◄────────TCP:80──────────────────► abb_rws_client (ROS2)  │
└─────────────────────────────────┘      └──────────────────────────────────┘
```

**Two separate channels:**
| Channel | Protocol | Port | Purpose |
|---|---|---|---|
| RWS | TCP | 80 | Motor on/off, program control, state |
| EGM | UDP | 6511 | Real-time joint position streaming ~250 Hz |

---

## Part 1 — VMware Network Setup

EGM requires the virtual controller and the ROS2 VM to share a **Host-Only** network (VMnet1).

### VM Network Adapters

The Ubuntu VM needs **two** network adapters in VMware settings:

| Adapter | VMware Type | Subnet | Purpose |
|---|---|---|---|
| ens33 | NAT (VMnet8) | 192.168.81.x | Internet access |
| ens37 | Host-Only (VMnet1) | 192.168.255.x | RobotStudio ↔ ROS2 |

**Windows Host addresses (auto-assigned by VMware):**
- `VMnet1: 192.168.255.1` — used by the virtual controller
- `VMnet8: 192.168.81.1`

Verify on the Ubuntu VM:
```bash
ip addr show ens37
# Should show: inet 192.168.255.128/24
```

Verify connectivity:
```bash
ping 192.168.255.1   # ping Windows host from VM
```

### Windows Firewall Rule

Allow inbound UDP on port 6511 (run once as Administrator in PowerShell):
```powershell
New-NetFirewallRule -DisplayName "ABB EGM UDP 6511" `
  -Direction Inbound -Protocol UDP -LocalPort 6511 `
  -Action Allow -Profile Any
```

---

## Part 2 — RobotStudio Virtual Controller Configuration

### 2.1 Install Required RobotWare Option

EGM requires the **Externally Guided Motion** option. Without it, `EGMGetId` silently fails.

> **Controller → Properties → System → Installed Options**

Look for `Externally Guided Motion (3124-1)`. If missing, rebuild the system:
> **Home → Robot System → New System** → check `Externally Guided Motion`

### 2.2 Communication → Transmission Protocol

> **Controller → Configuration → Communication → Transmission Protocol**

Add a UDPUC entry:

| Field | Value |
|---|---|
| Name | `UCdevice` |
| Type | `UDPUC` |
| Remote Address | `192.168.255.128` *(IP of ROS2 VM on VMnet1)* |
| Remote Port Number | `6511` |
| Local Port Number | `0` *(auto-assign)* |

### 2.3 Communication → IP Setting

> **Controller → Configuration → Communication → IP Setting**

Add entry:

| Field | Value |
|---|---|
| Name | `LAN` |
| IP Address | `192.168.255.1` *(VMnet1 host adapter IP)* |
| Subnet Mask | `255.255.255.0` |
| Interface | `LAN` |

### 2.4 Communication → IP Route

> **Controller → Configuration → Communication → IP Route**

Add entry:

| Field | Value |
|---|---|
| Destination | `192.168.255.0` |
| Gateway | `192.168.255.1` |
| Label | `LAN` |

### 2.5 Motion → External Motion Interface

> **Controller → Configuration → Motion → External Motion Interface**

Verify a `default` entry exists:

| Name | Level | Default Ramp Time | Default Proportional Position Gain |
|---|---|---|---|
| `default` | Filtering | 0.5 | 5 |

> This is the EGM config. The name `"default"` is referenced in RAPID's `EGMSetupUC`.

### 2.6 Apply and Restart Controller

After all configuration changes:
> **Controller → Restart → Reset (I-start)**

---

## Part 3 — RAPID EGM Module

File: `EGM_ROS2.mod` (load into `T_ROB1` task)

```rapid
MODULE EGM_ROS2

    VAR egmident egmID1;

    PROC main()
        EGMGetId egmID1;

        ! Bind to UCdevice (UDPUC -> 192.168.255.128:6511)
        ! "default"   = External Motion Interface config (Motion > Configuration)
        ! "UCdevice"  = Transmission Protocol entry (Communication > Configuration)
        EGMSetupUC ROB_1, egmID1, "default", "UCdevice" \Joint;

        EGMActJoint egmID1
            \Tool:=tool0
            \WObj:=wobj0
            \MaxPosDeviation:=1000
            \MaxSpeedDeviation:=1000;

        ! Loop forever — EGMRunJoint restarts automatically each cycle
        ! CondTime:=3600 = 1 hour per cycle (effectively continuous)
        WHILE TRUE DO
            EGMRunJoint egmID1, EGM_STOP_HOLD
                \J1 \J2 \J3 \J4 \J5 \J6
                \CondTime:=3600;
        ENDWHILE

        EGMStop egmID1, EGM_STOP_HOLD;
        EGMReset egmID1;

    ERROR
        EGMReset egmID1;
        RAISE;
    ENDPROC

ENDMODULE
```

### Key design decisions

| Decision | Reason |
|---|---|
| `VAR egmident` (not `PERS`) | `egmident` is a non-value type — `PERS` causes error 83 |
| No `EGMReset` before `EGMGetId` | `VAR` is uninitialized on first run — resetting it causes error 41820 |
| `EGMSetupUC` instead of `\UCdevice` arg | RobotWare 6.08+ removed `\UCdevice` from `EGMActJoint`; use the separate setup instruction |
| `WHILE TRUE` loop | Prevents EGM from stopping after `CondTime` expires |
| `ERROR` handler with `EGMReset` | Safely releases the EGM identity on any runtime error |

### Loading the module

> **RAPID tab → T_ROB1 → right-click → Load Module** → select `EGM_ROS2.mod`
> Then: **RAPID → Synchronize → Synchronize to Station**

---

## Part 4 — ROS2 Humble Setup

### 4.1 Prerequisites

```bash
# ROS2 Humble (assumed installed)
source /opt/ros/humble/setup.bash

# PickNik abb_ros2 workspace (assumed built)
source ~/ros2abb_ws/install/setup.bash

# Protobuf compiler
sudo apt install -y protobuf-compiler

# Python protobuf library
pip3 install protobuf
```

### 4.2 Compile EGM Protobuf Definitions

The proto files are in `abb_libegm` (part of PickNik's workspace):

```bash
cd ~/ros2abb_ws/src/abb_libegm/proto/

# Compile all three proto files (they cross-reference each other)
protoc --python_out=. egm.proto egm_wrapper.proto egm_wrapper_trajectory.proto

# Verify output
ls *_pb2.py
# egm_pb2.py  egm_wrapper_pb2.py  egm_wrapper_trajectory_pb2.py

# Copy to your script directory
cp *_pb2.py ~/ros2abb_ws/script/
```

### 4.3 EGM Joint Node

Save as `~/ros2abb_ws/script/egm_joint_node.py`:

```python
#!/usr/bin/env python3
"""
Model-agnostic ABB EGM joint stream node for ROS2 Humble.
No URDF or robot support package required.
Works with any ABB robot running EGMRunJoint in RAPID.

Topics:
  Published:   /egm/joint_states  [sensor_msgs/JointState]  — live feedback (radians)
  Subscribed:  /egm/joint_command [std_msgs/Float64MultiArray] — reference (degrees)
"""

import socket
import threading
import sys
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

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

import egm_pb2


class EGMJointNode(Node):
    def __init__(self):
        super().__init__('egm_joint_node')

        self.declare_parameter('egm_port',   6511)
        self.declare_parameter('bind_ip',    '0.0.0.0')
        self.declare_parameter('num_joints', 6)
        self.declare_parameter('joint_names', [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6',
        ])

        self.port       = self.get_parameter('egm_port').value
        self.bind_ip    = self.get_parameter('bind_ip').value
        self.num_joints = self.get_parameter('num_joints').value
        self.jnames     = self.get_parameter('joint_names').value[:self.num_joints]

        self._current_joints = [0.0] * self.num_joints
        self._ref_joints     = None   # None = echo back (hold position)
        self._lock = threading.Lock()
        self._seq  = 0

        self._pub_js = self.create_publisher(JointState, '/egm/joint_states', 10)
        self.create_subscription(Float64MultiArray, '/egm/joint_command', self._cmd_cb, 10)

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.bind_ip, self.port))
        self._sock.settimeout(1.0)

        self.get_logger().info(
            f"EGM node ready — listening on UDP {self.bind_ip}:{self.port} "
            f"| joints: {self.num_joints} ({', '.join(self.jnames)})"
        )

        self._running = True
        self._thread  = threading.Thread(target=self._udp_loop, daemon=True)
        self._thread.start()

    def _cmd_cb(self, msg: Float64MultiArray):
        with self._lock:
            self._ref_joints = list(msg.data[:self.num_joints])

    def _udp_loop(self):
        robot_msg  = egm_pb2.EgmRobot()
        sensor_msg = egm_pb2.EgmSensor()

        while self._running and rclpy.ok():
            try:
                data, addr = self._sock.recvfrom(4096)
            except socket.timeout:
                continue

            robot_msg.ParseFromString(data)
            fb = list(robot_msg.feedBack.joints.joints)[:self.num_joints]

            with self._lock:
                self._current_joints = fb
                ref = (self._ref_joints or fb)[:]

            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name         = self.jnames
            js.position     = [j * 3.14159 / 180.0 for j in fb]  # deg → rad
            self._pub_js.publish(js)

            sensor_msg.Clear()
            sensor_msg.header.seqno = self._seq
            sensor_msg.header.tm    = robot_msg.header.tm
            sensor_msg.planned.joints.joints[:] = ref   # EgmSensor.planned (not plannedTrajectory)
            self._sock.sendto(sensor_msg.SerializeToString(), addr)
            self._seq += 1

    def destroy_node(self):
        self._running = False
        self._sock.close()
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
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Part 5 — Startup Sequence

This is the correct multi-terminal workflow to initialize the robot, start the RAPID program via RWS, and stream EGM joint references.

**Terminal 1 (RWS Server & Client)**
Start the RWS client to enable state management. The `curl` command verifies the connection first.
```bash
# 1. Verify RWS connection
curl --digest -u "Default User:robotics" -c /tmp/abb.cookie -b /tmp/abb.cookie "http://192.168.255.1/rw/system" --max-time 5

# 2. Launch RWS client
ros2 launch abb_bringup abb_rws_client.launch.py \
  robot_ip:=192.168.255.1 \
  robot_port:=80 \
  robot_nickname:=irb460 \
  no_connection_timeout:=true \
  polling_rate:=5.0
```

**Terminal 2 (Initialize Robot State & Start EGM Node)**
Use RWS to prepare the controller, then start the Python EGM node. 

> **Important:** Do NOT launch `abb_robot_driver.launch.py`. The Python `egm_joint_node.py` is a standalone EGM client that replaces it! Running both will cause a port conflict on UDP 6511.

```bash
# 1. Turn on motors
ros2 service call /rws_client/set_motors_on abb_robot_msgs/srv/TriggerWithResultCode "{}"

# 2. Reset program pointer to main
ros2 service call /rws_client/pp_to_main abb_robot_msgs/srv/TriggerWithResultCode "{}"

# 3. Start RAPID execution (EGMRunJoint will block and wait for UDP packets)
ros2 service call /rws_client/start_rapid abb_robot_msgs/srv/TriggerWithResultCode "{}"

# 4. Start the EGM Python node (opens UDP 6511 and establishes the stream)
python3 egm_joint_node.py \
  --ros-args \
  -p num_joints:=4 \
  -p joint_names:="['joint_1','joint_2','joint_3','joint_4']"
```

**Terminal 3 (Publish Commands)**
Send references to the robot. *(Note: EGM expects smooth trajectories. Instantaneous jumps like `10.0` degrees might trigger a speed deviation abort in the controller. Use `rqt_publisher` for smooth waves).*
```bash
ros2 topic pub /egm/joint_command std_msgs/msg/Float64MultiArray "data: [10.0, -20.0, 15.0, 0.0]"
```

**Terminal 4 (Monitor Feedback)**
Watch the live robot joints coming back from the controller.
```bash
ros2 topic echo /egm/joint_states
```

---

## Part 6 — Verification Commands

```bash
# Is anything listening on UDP 6511?
ss -ulnp | grep 6511

# Are EGM packets arriving from the controller?
sudo tcpdump -i ens37 udp port 6511 -n

# Live joint feedback
ros2 topic echo /egm/joint_states

# Stream rate (expect ~250 Hz)
ros2 topic hz /egm/joint_states

# Current joint positions (raw degrees)
ros2 topic echo /egm/joint_states --once
```

---

## Part 7 — Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Error 83: `egmident not value type` | `PERS egmident` declaration | Change to `VAR egmident` |
| Error 41820: `Invalid EGM identity` | `EGMReset` called before `EGMGetId`, or EGM option not installed | Remove leading `EGMReset`; check installed RobotWare options for `3124-1` |
| `\UCdevice` invalid argument | RobotWare 6.08+ removed it from `EGMActJoint` | Use `EGMSetupUC` instruction instead |
| `AttributeError: plannedTrajectory` | Wrong proto field name | Use `sensor_msg.planned` (not `plannedTrajectory`) |
| `ss -ulnp` shows nothing on :6511 | `abb_robot_driver` launch failed or EGM node not started | Start `egm_joint_node.py` manually |
| tcpdump shows no packets | Virtual controller not bound to VMnet1 | Add IP Setting `192.168.255.1` to Communication config; restart controller |
| EGM times out after 60s | `\CondTime:=60` expired | Use `WHILE TRUE DO EGMRunJoint ... ENDWHILE` loop |
| `abb_robot_driver` package not found | PickNik repo uses `abb_bringup`, not `abb_robot_driver` | Use `ros2 launch abb_bringup ...` or run `egm_joint_node.py` directly |

---

## Part 8 — EGM Proto Message Structure Reference

Key fields used from `~/ros2abb_ws/src/abb_libegm/proto/egm.proto`:

```
EgmRobot  (controller → ROS2, inbound)
  └── feedBack: EgmFeedBack
        └── joints: EgmJoints
              └── joints: repeated double   ← actual joint positions (degrees)

EgmSensor  (ROS2 → controller, outbound)
  └── planned: EgmPlanned
        └── joints: EgmJoints
              └── joints: repeated double   ← reference joint positions (degrees)
```

> **Note:** Positions are in **degrees** in the EGM protocol (ABB convention).  
> Convert to radians for ROS2: `rad = deg * π / 180`

---

## File Locations

| File | Location |
|---|---|
| RAPID module (EGM) | `C:\Users\Admin\Documents\RobotStudio\Projects\ROS2ABB\User Files\EGM_ROS2.mod` |
| RAPID module (RWS Control) | `C:\Users\Admin\Documents\Programs\RAPIDABB\ROS2_Control.mod` |
| RAPID module (Hybrid) | `C:\Users\Admin\Documents\Programs\RAPIDABB\Hybrid_Control.mod` |
| EGM ROS2 node | `~/ros2abb_ws/script/egm_joint_node.py` |
| RWS bootstrap (Python) | `~/ros2abb_ws/script/abb_start.py` |
| Compiled proto | `~/ros2abb_ws/script/egm_pb2.py` |
| Proto source | `~/ros2abb_ws/src/abb_libegm/proto/egm.proto` |

---

## Part 9 — RWS-Only Middleware (Proven Working Alternative to EGM)

> **Status:** ✅ Successfully validated. This is the recommended starting architecture for most palletizing tasks.

### 9.1 Why RWS Instead of EGM

EGM requires tight timing, correct protobuf headers, and the controller must never lose the 250 Hz UDP heartbeat. For the IRB460 virtual controller, several compounding issues made EGM difficult to bring up reliably:

- `abb_rws_client` subscription leak → "Number of instance values exceeded" error
- `curl` Digest Auth POST body drop bug → Motors ON and Start RAPID silently failed
- EGM joint mapping mismatch: user J4 must map to physical J6 in the 6-element proto array

RWS-only architecture avoids all of the above. The robot uses its own ABB motion planner (`MoveAbsJ`), and ROS2 acts as the orchestration brain.

### 9.2 ROS2_Control.mod — The Proven RAPID Module

```rapid
MODULE ROS2_Control

    PERS jointtarget ros_target := [[45,20,10,0,0,0],[9E+9,9E+9,9E+9,9E+9,9E+9,9E+9]];
    PERS bool ros_execute := FALSE;

    PROC main()
        WHILE TRUE DO
            IF ros_execute THEN
                MoveAbsJ ros_target, v100, z50, tool0;
                ros_execute := FALSE;
            ENDIF
            WaitTime 0.05;
        ENDWHILE
    ENDPROC

ENDMODULE
```

**Key design decisions:**
- `PERS` variables survive RAPID restarts and are readable/writable via RWS HTTP API
- `ros_execute` acts as an edge-trigger: ROS sets it `TRUE`, RAPID clears it after the move completes
- `WaitTime 0.05` (20 Hz polling) is sufficient — no real-time streaming needed

### 9.3 ROS2 Service Calls to Drive the Robot

#### Set the target joint position:
```bash
ros2 service call /rws_client/set_rapid_symbol \
  abb_robot_msgs/srv/SetRAPIDSymbol \
  '{path: {task: "T_ROB1", module: "ROS2_Control", symbol: "ros_target"},
    value: "[[45.0,20.0,10.0,0.0,0.0,0.0],[9E9,9E9,9E9,9E9,9E9,9E9]]"}'
```

#### Trigger execution:
```bash
ros2 service call /rws_client/set_rapid_bool \
  abb_robot_msgs/srv/SetRAPIDBool \
  '{path: {task: "T_ROB1", module: "ROS2_Control", symbol: "ros_execute"},
    value: true}'
```

> **Note:** `ros_target` joint array maps directly to controller joints `[J1,J2,J3,J4,J5,J6]`.  
> For the IRB460 palletizer, J4 and J5 are locked (ignored by `MoveAbsJ`).  
> The wrist rotation goes in index 5 (J6): `[j1, j2, j3, 0, 0, j6_wrist]`.

### 9.4 Monitoring — What the RWS Client Already Provides

No custom RAPID code is needed for basic monitoring. The `abb_rws_client` node publishes these automatically:

| ROS2 Topic | Content | Rate |
|---|---|---|
| `/rws_client/system_states` | Motor state, RAPID running state | On-change |
| `/rws_client/joint_states` | Joint positions (degrees, polled) | ~10 Hz |
| `/rws_client/robot_controller_state` | AUTO / MANUAL mode | On-change |

#### Optional: Add application-level status to RAPID
If your orchestration logic needs to know when a move is complete, add these PERS fields to your RAPID module:

```rapid
PERS string ros_status := "IDLE";   ! "IDLE" | "MOVING" | "DONE"
PERS num ros_move_count := 0;       ! Increments every completed move
```

Poll them from ROS2:
```bash
ros2 service call /rws_client/get_rapid_symbol \
  abb_robot_msgs/srv/GetRAPIDSymbol \
  '{path: {task: "T_ROB1", module: "ROS2_Control", symbol: "ros_status"}}'
```

### 9.5 Bootstrap: Starting RAPID Without the RWS Client Subscription Overflow

The `abb_rws_client` creates persistent WebSocket event subscriptions that fill up the controller's instance table if repeatedly launched without a clean stop. Use `abb_start.py` instead for initial setup:

```bash
# On Ubuntu VM
python3 abb_start.py
```

This uses Python's `urllib` (Digest Auth, no curl bug) to:
1. Authenticate
2. Request Mastership
3. Set Motors ON
4. Reset PP to Main
5. Start RAPID (with `execmode=continuous`)
6. Release Mastership

> **Critical lesson:** `curl` with Digest Auth drops POST body on the challenge-response retry.  
> Always use Python `urllib` or `requests` for RWS POST requests with a body.

### 9.6 Hybrid Control: RWS to Safe Pose → EGM Streaming

If EGM is still desired, use `Hybrid_Control.mod` to sequence the startup safely:

```
ROS2 → set ros_target → trigger execute_ros_move
  → Robot moves to safe pose via MoveAbsJ (ABB planner)
    → ROS2 sets start_egm := TRUE
      → RAPID calls EGMRunJoint → egm_joint_node.py takes over
```

This avoids EGM startup failures caused by the robot sitting at a kinematic singularity (all-zeros joint pose).

### 9.7 Joint Mapping Summary (IRB460 Palletizer)

| Logical Axis | ROS2 Joint Name | Proto Array Index | Controller Joint | Notes |
|---|---|---|---|---|
| J1 | `joint_1` | 0 | J1 | Base rotation |
| J2 | `joint_2` | 1 | J2 | Shoulder |
| J3 | `joint_3` | 2 | J3 | Elbow |
| J4 (user) | `joint_6` | 5 | J6 | Wrist (palletizer maps user J4 → physical J6) |
| — | — | 3 | J4 | **LOCKED** in controller config |
| — | — | 4 | J5 | **LOCKED** in controller config |

> Confirmed via RobotStudio **Configuration → Motion → Robot** table:  
> `Use Joint 4: LOCKED_rob1_4`, `Use Joint 5: LOCKED_rob1_5`

