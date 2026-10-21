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
#from typeguard import value

import rclpy
from rclpy.node import Node
from rov_interfaces.msg import MotionCommands , ActuatorsCommands , AutonomousCommand


class joystick(Node):
    def __init__(self):
        super().__init__("joystick")
        self.pub = self.create_publisher(MotionCommands, "/joystick_motion", 10)
        self.create_timer(0.1, self.read_joystick)

        self.joystick_publisher=self.create_publisher(ActuatorsCommands,"/joystick_actuators",10)
        self.create_timer(0.1,self.read_joystick)

        self.autonomous_publisher=self.create_publisher(AutonomousCommand,"/autonomous_joystick",10)
        self.create_timer(0.1,self.read_joystick)

        #for toggle buttons:
        self.p1_state=False
        self.p2_state=False
        self.p3_state=False
        self.p4_state=False
        self.p5_state=False
        self.p6_state=False
        self.motor_state=False
        self.dpad_up_pressed = False
        self.dpad_down_pressed = False
        self.auto=False
        # e.BTN_START: 21,   # Options




    def read_joystick(self):

        self.keys = [0] * 25

        # one-shot buttons: gamepad button -> index in the array
        ONE_SHOT = {
          
            e.BTN_TL: 18,      # L1
            e.BTN_TR: 19,      # R1
            e.BTN_SELECT: 20,  # Share
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
                return min(100, int((offset - DEADZONE) * 100 / (127 - DEADZONE)))
            
            if code == e.ABS_RZ:
                self.keys[0] = int(value *100/255)    # R2
            elif code == e.ABS_Z:
                self.keys[1] = int(value *100/255)      # L2
            elif code == e.ABS_HAT0X:
                self.keys[2] = int(value == 1)             # D-pad right
                self.keys[3] = int(value == -1)            # D-pad left
                            
            elif code == e.ABS_HAT0Y:
                if value == -1 and not self.dpad_up_pressed:
                    self.p5_state = not self.p5_state
                if value == 1 and not self.dpad_down_pressed:
                    self.p6_state = not self.p6_state
                self.dpad_up_pressed = (value == -1)   
                self.dpad_down_pressed = (value == 1)            
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
                elif ev.type == e.EV_KEY and ev.value == 1:
                    #for toggling certain buttons
                    if ev.code == e.BTN_EAST:
                        self.p1_state = not self.p1_state

                    elif ev.code == e.BTN_NORTH:
                        self.p2_state = not self.p2_state

                    elif ev.code == e.BTN_WEST:
                        self.p3_state = not self.p3_state

                    elif ev.code == e.BTN_SOUTH:
                        self.p4_state = not self.p4_state

                    
                    elif ev.code == e.BTN_MODE:
                        self.motor_state = not self.motor_state

                    elif ev.code == e.BTN_START: 
                        self.auto = not self.auto
                    
                    elif ev.code in ONE_SHOT:
                        with lock:
                            self.pending.add(ONE_SHOT[ev.code])

    
        pad = find_gamepad(self)
        if pad is None:
            self.get_logger().error("Gamepad not found. Connect your gamepad first.")
            rclpy.shutdown()   # stops spin() so the node exits cleanly
            return

        threading.Thread(target=read_gamepad, args=(pad,), daemon=True).start()

        try:
            while True:
                # one-shot buttons: 1 for this refresh only, then back to 0
                with lock:
                    for i in ONE_SHOT.values():
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
                motion_msg.depth_hold=bool(self.keys[24])

                
                self.pub.publish(motion_msg)



                actuators = ActuatorsCommands()

                actuators.p1 = self.p1_state
                actuators.p2 = self.p2_state
                actuators.p3 = self.p3_state
                actuators.p4 = self.p4_state
                actuators.p5 = self.p5_state
                actuators.p6 = self.p6_state
                
                actuators.motor = self.motor_state

                self.joystick_publisher.publish(actuators)

                automation = AutonomousCommand()
                automation.autonomous_mode = self.auto
                self.autonomous_publisher.publish(automation)





                


                time.sleep(0.1)   # 100 ms
        except KeyboardInterrupt:
            print("\nStopped.")

def main():
    rclpy.init()
    node = joystick()
    rclpy.spin(node)
    
if __name__ == "__main__":
    main()