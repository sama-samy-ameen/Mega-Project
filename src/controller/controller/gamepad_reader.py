#!/usr/bin/env python3
"""
Reads the virtual PS4 gamepad made by ps4_keyboard.py and keeps a 25-item
0/1 array up to date. The array is refreshed every 100 ms.

HOLDABLE (1 for as long as it is held, then back to 0):
  0 R2             4 Left stick up       8 Right stick up
  1 L2             5 Left stick down     9 Right stick down
  2 D-pad right    6 Left stick right   10 Right stick right
  3 D-pad left     7 Left stick left    11 Right stick left

ONE-SHOT (1 for a single refresh when pressed, then back to 0):
  12 D-pad up   15 Circle     18 L1      21 Options   24 R3
  13 D-pad down 16 Triangle   19 R1      22 PS
  14 Cross      17 Square     20 Share   23 L3

Run:   sudo python3 gamepad_reader.py
Stop:  Ctrl+C
"""

import sys
import threading
import time
import numpy

from evdev import InputDevice, list_devices, ecodes as e

import rclpy
from rclpy.node import Node
from rov_interfaces.msg import MotionCommands , ActuatorsCommands


class joystick(Node):
    def __init__(self):
        super().__init__("joystick")
        self.pub = self.create_publisher(MotionCommands, "/joystick_motion", 10)
        self.create_timer(0.1, self.read_joystick)

        self.joystick_publisher=self.create_publisher(ActuatorsCommands,"/joystick_actuators",10)
        self.create_timer(0.1,self.read_joystick)

        #for toggle
        self.p1_state = False
        self.p2_state = False
        self.p3_state = False
        self.p4_state = False



    def read_joystick(self):

        self.keys = [0] * 25

        # one-shot buttons: gamepad button -> index in the array
        ONE_SHOT = {
            e.BTN_SOUTH: 14,   # Cross
            e.BTN_EAST: 15,    # Circle
            e.BTN_NORTH: 16,   # Triangle
            e.BTN_WEST: 17,    # Square
            e.BTN_TL: 18,      # L1
            e.BTN_TR: 19,      # R1
            e.BTN_SELECT: 20,  # Share
            e.BTN_START: 21,   # Options
            e.BTN_MODE: 22,    # PS
            e.BTN_THUMBL: 23,  # L3
            e.BTN_THUMBR: 24,  # R3
        }

        self.pending = set()           # one-shot presses waiting for the next refresh
        lock = threading.Lock()

        def find_gamepad(self):
            for path in list_devices():
                dev = InputDevice(path)
                caps = dev.capabilities()
                # a gamepad has face buttons and stick axes, whatever its name is
                if e.BTN_SOUTH in caps.get(e.EV_KEY, []) and \
                   e.ABS_X in [code for code, _ in caps.get(e.EV_ABS, [])]:
                    return dev
            return None


        def update(code, value):
            """Turn one gamepad axis event into values (triggers/sticks are 0-255)."""
            DEADZONE = 30

            def push(value, direction):
                """How far the stick is pushed one way: 0 inside the deadzone, up to 255."""
                offset = (value - 128) * direction
                if offset <= DEADZONE:
                    return 0
                return min(255, int((offset - DEADZONE) * 127 / (127 - DEADZONE)))

            if code == e.ABS_RZ:
                self.keys[0] = int(value *127/255)    # R2
            elif code == e.ABS_Z:
                self.keys[1] = int(value *127/255)      # L2
            elif code == e.ABS_HAT0X:
                self.keys[2] = int(value == 1)             # D-pad right
                self.keys[3] = int(value == -1)            # D-pad left
            elif code == e.ABS_HAT0Y:                      # D-pad up/down are one-shot
                if value == -1:
                    with lock:
                        self.pending.add(12)               # D-pad up
                elif value == 1:
                    with lock:
                        self.pending.add(13)               # D-pad down
            elif code == e.ABS_Y:
                self.keys[4] = push(value, -1)             # Left stick up
                self.keys[5] = push(value, 1)              # Left stick down
            elif code == e.ABS_X:
                self.keys[6] = push(value, 1)              # Left stick right
                self.keys[7] = push(value, -1)             # Left stick left
            elif code == e.ABS_RY:
                self.keys[8] = push(value, -1)             # Right stick up
                self.keys[9] = push(value, 1)              # Right stick down
            elif code == e.ABS_RX:
                self.keys[10] = push(value, 1)             # Right stick right
                self.keys[11] = push(value, -1)            # Right stick left

        def read_gamepad(pad):
            for ev in pad.read_loop():
                if ev.type == e.EV_ABS:
                    update(ev.code, ev.value)
                elif ev.type == e.EV_KEY and ev.value == 1 and ev.code in ONE_SHOT:
                    with lock:
                        self.pending.add(ONE_SHOT[ev.code])   # remember the press


    
        pad = find_gamepad(self)
        if pad is None:
            self.get_logger().error("Gamepad not found. Connect your gamepad first.")
            rclpy.shutdown()   # stops spin() so the node exits cleanly
            return

        threading.Thread(target=read_gamepad, args=(pad,), daemon=True).start()
                with self.lock:
            # Toggle piston state once per press.
            if 15 in self.pending:       # Circle -> p1
                self.p1_state = not self.p1_state
            if 16 in self.pending:       # Triangle -> p2
                self.p2_state = not self.p2_state
            if 17 in self.pending:       # Square -> p3
                self.p3_state = not self.p3_state
            if 14 in self.pending:       # Cross -> p4
                self.p4_state = not self.p4_state

            # One-refresh commands, including D-pad Up/Down.
            for i in [12, 13] + list(self.one_shot.values()):
                self.keys[i] = 1 if i in self.pending else 0

            self.pending.clear()

            # Copy shared values while holding the lock.
            keys = self.keys.copy()
            p1 = self.p1_state
            p2 = self.p2_state
            p3 = self.p3_state
            p4 = self.p4_state

        try:
            while True:
                # one-shot buttons: 1 for this refresh only, then back to 0
                with lock:
                    for i in [12, 13] + list(ONE_SHOT.values()):
                        self.keys[i] = 1 if i in self.pending else 0
                    self.pending.clear()

                motion_msg = MotionCommands()

                motion_msg.speeddown = bool(self.keys[18])
                motion_msg.speedup = bool(self.keys[19])

                # motion_msg.surgeforward = self.keys[0]
                # motion_msg.surgebackward = self.keys[1]
                motion_msg.surge = self.keys[0] - self.keys[1]

                # motion_msg.rollcw = self.keys[2]
                # motion_msg.rollccw = self.keys[3]
                motion_msg.roll = self.keys[3] - self.keys[2]

                # motion_msg.heaveup = self.keys[4]
                # motion_msg.heavedown = self.keys[5]
                motion_msg.heave = self.keys[4] - self.keys[5]

                # motion_msg.yawcw = self.keys[6]
                # motion_msg.yawccw = self.keys[7]
                motion_msg.yaw = self.keys[7] - self.keys[6]

                # motion_msg.pitchup = self.keys[8]
                # motion_msg.pitchdown = self.keys[9]
                motion_msg.pitch = self.keys[8] - self.keys[9]

                # motion_msg.swayright = self.keys[10]
                # motion_msg.swayleft = self.keys[11]
                motion_msg.sway = self.keys[10] - self.keys[11]

                
                self.pub.publish(motion_msg)



                actuators_msg = ActuatorsCommands()

                actuators_msg.p1 = bool(self.keys[15])      # p1  circle

                actuators_msg.p2 = bool(self.keys[16])      # p2  triangle

                actuators_msg.p3 = bool(self.keys[17])      # p3  square

                actuators_msg.p4 = bool(self.keys[14])      # p4  x

                actuators_msg.dir1 = bool(self.keys[12])    # motor_dir1  d_up

                actuators_msg.dir2 = bool(self.keys[13])    # motor_dir2  d_down

                self.joystick_publisher.publish(actuators_msg)



                time.sleep(0.1)   # 100 ms
        except KeyboardInterrupt:
            print("\nStopped.")

def main():
    rclpy.init()
    node = joystick()
    rclpy.spin(node)
    
if __name__ == "__main__":
    main()
