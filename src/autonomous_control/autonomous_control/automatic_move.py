#!/usr/bin/env python3
"""
Autonomous control node (FSM)

  SEARCH -> STABILIZE -> CENTER -> APPROACH -> LEVEL -> SHIFT_FORWARD -> SHIFT_SIDE -> RELEASE -> DONE
    1                      2          3         4A          4B              4C           4D

Commands go out on /autonomous_command ONLY while /rov/serial says autonomous_mode == True.
"""

import rclpy
from rclpy.node import Node
from enum import Enum, auto
from collections import deque
import numpy as np

from sensor_msgs.msg import Imu
from rov_interfaces.msg import ModelReadings, AutonomousCommand
from rov_interfaces.msg import RovSerial   # <-- ASSUMPTION: use your real '/rov/serial' msg type


# =====================================================================
#                      TUNING  -  EDIT HERE ONLY
#   (every command value is a SIGNED OFFSET from NEUTRAL, see below)
# =====================================================================

# --- Command encoding ------------------------------------------------
# AutonomousCommand fields are uint8, so "negative" needs a centre value.
# ASSUMPTION: NEUTRAL = stop, above it = positive direction, below = negative.
NEUTRAL = 127

# --- Cameras / frame -------------------------------------------------
OAK_CAMERA = 1                        # camera id of the OAK-D (0 = target not seen)
OAK_WIDTH, OAK_HEIGHT = 1920, 1080    # frame size that the model's x / y are in

# Webcams 2-5: (yaw sign, pitch sign) used to turn toward the OAK
CAMERA_TURN = {
    2: (+1, +1),    # angular_z+, angular_y+
    3: (-1, +1),    # angular_z-, angular_y+
    4: (+1, -1),    # angular_z+, angular_y-
    5: (-1, -1),    # angular_z-, angular_y-
}
TURN_CMD = 15                         # slow turn while only a webcam sees the target

# --- 1. Search ("ballerina": spin + rise) ----------------------------
SEARCH_YAW_CMD = 15
SEARCH_RISE_CMD = 10
SEARCH_RISE_MAX_S = 5.0               # stop rising after this long (keeps spinning)

# --- Stabilization (right after the target is first found) -----------
STABLE_WINDOW = 5                     # IMU samples kept (5 x 100 ms = 0.5 s)
STABLE_STD_DEG = 1.0                  # max roll/pitch std-dev to call it "steady"
STABLE_MIN_S = 1.0                    # always wait at least this long
STABLE_MAX_S = 3.0                    # give up waiting (IMU is only a rough check)

# --- 2. Centering on the OAK (PID: pixel error -> command units) -----
YAW_GAINS = (0.03, 0.002, 0.01)       # (kp, ki, kd)   <-- untuned starting values
PITCH_GAINS = (0.03, 0.002, 0.01)
PID_MAX_CMD = 30                      # clamp of the PID output
YAW_DIR = +1                          # flip to -1 if it turns AWAY from the target (left/right)
PITCH_DIR = +1                        # flip to -1 if it pitches AWAY from the target (up/down)
CENTER_TOL_PX = 40                    # "centered" = |error| below this on both axes
CENTER_HOLD_S = 0.5                   # ...for this long, then move on

# --- 3. Approach -----------------------------------------------------
APPROACH_CMD = 10                     # slow linear_x
TARGET_WIDTH_PX = 100                 # wanted target size in the OAK frame
TARGET_HEIGHT_PX = 100
SIZE_TOL_PX = 10                      # done when width OR height is within +/- this

# --- 4. Leave the object ---------------------------------------------
LEVEL_KP = 2.0                        # pitch error (deg) -> angular_y command
LEVEL_DIR = +1                        # flip to -1 if pitch gets worse instead of better
LEVEL_TOL_DEG = 3.0                   # "pitch == 0" within this
LEVEL_MAX_CMD = 30
LEVEL_TIMEOUT_S = 3.0                 # continue anyway after this long (IMU is rough)
SHIFT_FORWARD_CMD = 50                # 4B
SHIFT_FORWARD_S = 0.25
SHIFT_SIDE_CMD = 50                   # 4C (sign flips automatically for gripper 2)
SHIFT_SIDE_S = 0.25
GRIPPER_ID = 1                        # 1 or 2
# gripper byte = (0,0,0,0, extend1,open1,extend2,open2)
# ASSUMPTION: a set "open" bit releases (your old let_go() sent 0b00000000)
GRIPPER_OPEN = {1: 0b00000100, 2: 0b00000001}
RELEASE_S = 1.0                       # how long the open bit is held

# --- Timing / safety -------------------------------------------------
CONTROL_PERIOD_S = 0.05               # 20 Hz loop (timed moves are quantized to this)
TARGET_STALE_S = 0.5                  # model data older than this counts as "not seen"
LOST_TIMEOUT_S = 1.0                  # target gone this long during CENTER/APPROACH -> SEARCH

# =====================================================================
#                          END OF TUNING
# =====================================================================


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def make_cmd(lx=0, ly=0, lz=0, ax=0, ay=0, az=0, gripper=0):
    """Signed offsets from NEUTRAL -> uint8 AutonomousCommand. make_cmd() = stop."""
    to_u8 = lambda v: int(clamp(round(NEUTRAL + v), 0, 255))
    cmd = AutonomousCommand()
    cmd.linear_x = to_u8(lx)
    cmd.linear_y = to_u8(ly)
    cmd.linear_z = to_u8(lz)
    cmd.angular_x = to_u8(ax)
    cmd.angular_y = to_u8(ay)
    cmd.angular_z = to_u8(az)
    cmd.gripper = gripper
    return cmd


class PID:
    def __init__(self, kp, ki, kd, limit):
        self.kp, self.ki, self.kd, self.limit = kp, ki, kd, limit
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_error = None

    def update(self, error, dt):
        self.integral += error * dt
        if self.ki:                                    # anti-windup
            i_max = self.limit / self.ki
            self.integral = clamp(self.integral, -i_max, i_max)
        derivative = 0.0 if self.prev_error is None else (error - self.prev_error) / dt
        self.prev_error = error
        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        return clamp(out, -self.limit, self.limit)


class State(Enum):
    SEARCH = auto()         # 1.  ballerina until the target is seen
    STABILIZE = auto()      #     hold still after the first detection
    CENTER = auto()         # 2.  put the target in the middle of the OAK
    APPROACH = auto()       # 3.  creep forward until the size matches
    LEVEL = auto()          # 4A. pitch back to 0
    SHIFT_FORWARD = auto()  # 4B
    SHIFT_SIDE = auto()     # 4C
    RELEASE = auto()        # 4D
    DONE = auto()           #     hold still until autonomous mode is toggled off/on


class AutonomousControlNode(Node):
    def __init__(self):
        super().__init__('autonomous_control_node')

        # FSM
        self.publish_available = False    # True while autonomous_mode is on
        self.state = State.DONE
        self.state_start = 0.0
        self.centered_ticks = 0
        self.last_seen = 0.0

        # Latest sensor data
        self.target = None                # ObjectData: x, y, width, hight, confidence, camera
        self.target_stamp = 0.0
        self.pitch = None                 # deg
        self.imu_window = deque(maxlen=STABLE_WINDOW)   # (roll, pitch) samples

        self.yaw_pid = PID(*YAW_GAINS, PID_MAX_CMD)
        self.pitch_pid = PID(*PITCH_GAINS, PID_MAX_CMD)

        self.handlers = {
            State.SEARCH: self.do_search,
            State.STABILIZE: self.do_stabilize,
            State.CENTER: self.do_center,
            State.APPROACH: self.do_approach,
            State.LEVEL: self.do_level,
            State.SHIFT_FORWARD: self.do_shift_forward,
            State.SHIFT_SIDE: self.do_shift_side,
            State.RELEASE: self.do_release,
            State.DONE: self.do_done,
        }

        # ROS I/O
        self.create_subscription(RovSerial, '/rov/serial', self.serial_callback, 10)
        self.create_subscription(ModelReadings, '/model_readings', self.model_callback, 10)
        self.create_subscription(Imu, '/imu_control', self.imu_callback, 10)
        self.command_pub = self.create_publisher(AutonomousCommand, '/autonomous_command', 10)
        self.create_timer(CONTROL_PERIOD_S, self.control_loop)

        self.get_logger().info('Autonomous Control Node Started.')

    # ------------------------------------------------------------------
    # Callbacks (store data only; all decisions happen in control_loop)
    # ------------------------------------------------------------------
    def serial_callback(self, msg):
        autonomous_mode = bool(msg.autonomous_mode)    # <-- ASSUMPTION: adjust field name
        if autonomous_mode and not self.publish_available:
            self.start_mission()
        self.publish_available = autonomous_mode

    def model_callback(self, msg):
        self.target = msg.targ_1
        self.target_stamp = self.now()

    def imu_callback(self, msg: Imu):
        # ASSUMPTION: roll / pitch / yaw (deg) arrive in orientation.x / y / z.
        # Yaw is deliberately unused (unreliable). Change the parsing here only.
        roll, self.pitch = msg.orientation.x, msg.orientation.y
        self.imu_window.append((roll, self.pitch))

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def control_loop(self):
        if not self.publish_available:
            return

        if self.target_camera() != 0:
            self.last_seen = self.now()
        if self.state in (State.CENTER, State.APPROACH) and self.now() - self.last_seen > LOST_TIMEOUT_S:
            self.get_logger().warn('Target lost')
            self.go(State.SEARCH)

        self.command_pub.publish(self.handlers[self.state]())

    # ------------------------------------------------------------------
    # FSM helpers
    # ------------------------------------------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def elapsed(self):
        return self.now() - self.state_start

    def go(self, new_state):
        self.get_logger().info(f'{self.state.name} -> {new_state.name}')
        self.state = new_state
        self.state_start = self.now()
        self.centered_ticks = 0
        self.yaw_pid.reset()
        self.pitch_pid.reset()

    def target_camera(self):
        """Camera id that sees the target (0 = none). Stale model data counts as none."""
        if self.target is None or self.now() - self.target_stamp > TARGET_STALE_S:
            return 0
        return self.target.camera

    def start_mission(self):
        self.get_logger().info('Autonomous mode ON')
        self.last_seen = self.now()
        # 1. is the item already in a camera?  yes -> center, no -> search
        self.go(State.CENTER if self.target_camera() != 0 else State.SEARCH)

    def timed_move(self, next_state, duration, **axes):
        """Send `axes` for `duration` seconds, then switch to next_state."""
        if self.elapsed() >= duration:
            self.go(next_state)
            return make_cmd()
        return make_cmd(**axes)

    # ------------------------------------------------------------------
    # State handlers: each returns the command for this tick
    # ------------------------------------------------------------------
    def do_search(self):
        if self.target_camera() != 0:
            self.get_logger().info('Target detected')
            self.go(State.STABILIZE)
            return make_cmd()
        rising = self.elapsed() < SEARCH_RISE_MAX_S
        return make_cmd(az=SEARCH_YAW_CMD, lz=SEARCH_RISE_CMD if rising else 0)

    def do_stabilize(self):
        t = self.elapsed()
        steady = (len(self.imu_window) == STABLE_WINDOW
                  and np.std(self.imu_window, axis=0).max() < STABLE_STD_DEG)
        if (t >= STABLE_MIN_S and steady) or t >= STABLE_MAX_S:
            self.go(State.CENTER)
        return make_cmd()                                  # thrusters off while settling

    def do_center(self):
        cam = self.target_camera()
        if cam == 0:
            return make_cmd()                              # hold; control_loop handles "lost"

        if cam != OAK_CAMERA:                              # a webcam sees it: turn toward the OAK
            yaw_sign, pitch_sign = CAMERA_TURN.get(cam, (0, 0))
            return make_cmd(az=yaw_sign * TURN_CMD, ay=pitch_sign * TURN_CMD)

        # OAK sees it: PID until x, y are at the frame centre
        err_x = self.target.x - OAK_WIDTH / 2
        err_y = self.target.y - OAK_HEIGHT / 2
        az = YAW_DIR * self.yaw_pid.update(err_x, CONTROL_PERIOD_S)
        ay = PITCH_DIR * self.pitch_pid.update(err_y, CONTROL_PERIOD_S)

        centered = abs(err_x) < CENTER_TOL_PX and abs(err_y) < CENTER_TOL_PX
        self.centered_ticks = self.centered_ticks + 1 if centered else 0
        if self.centered_ticks * CONTROL_PERIOD_S >= CENTER_HOLD_S:
            self.go(State.APPROACH)
            return make_cmd()
        return make_cmd(az=az, ay=ay)

    def do_approach(self):
        cam = self.target_camera()
        if cam == 0:
            return make_cmd()                              # hold; control_loop handles "lost"
        if cam != OAK_CAMERA:
            self.go(State.CENTER)
            return make_cmd()

        # map says width OR height (your old code required both)
        size_ok = (abs(self.target.width - TARGET_WIDTH_PX) < SIZE_TOL_PX
                   or abs(self.target.hight - TARGET_HEIGHT_PX) < SIZE_TOL_PX)
        if size_ok:
            self.go(State.LEVEL)
            return make_cmd()
        return make_cmd(lx=APPROACH_CMD)

    def do_level(self):
        timed_out = self.elapsed() >= LEVEL_TIMEOUT_S
        if self.pitch is None or abs(self.pitch) < LEVEL_TOL_DEG or timed_out:
            if timed_out:
                self.get_logger().warn('Pitch levelling timed out, continuing')
            self.go(State.SHIFT_FORWARD)
            return make_cmd()
        ay = clamp(LEVEL_DIR * LEVEL_KP * (0.0 - self.pitch), -LEVEL_MAX_CMD, LEVEL_MAX_CMD)
        return make_cmd(ay=ay)

    def do_shift_forward(self):
        return self.timed_move(State.SHIFT_SIDE, SHIFT_FORWARD_S, lx=SHIFT_FORWARD_CMD)

    def do_shift_side(self):
        side = SHIFT_SIDE_CMD if GRIPPER_ID == 1 else -SHIFT_SIDE_CMD
        return self.timed_move(State.RELEASE, SHIFT_SIDE_S, ly=side)

    def do_release(self):
        return self.timed_move(State.DONE, RELEASE_S, gripper=GRIPPER_OPEN[GRIPPER_ID])

    def do_done(self):
        return make_cmd()


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()