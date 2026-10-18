import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int16MultiArray, Float32
from rov_interfaces.msg import MotionCommands

class Motion(Node):
    def __init__(self):
        super().__init__("motion_node")
        self.speed = 20
        self.max_power = 200
        self.speeds = []
        self.create_subscription(MotionCommands, "/joystick_motion", self.joystick_motion, 10)

    def joystick_motion(self, msg:MotionCommands):

        if msg.speedup and self.speed < 200:
            self.speed += 20
        if msg.speeddown and self.speed > 20:
            self.speed -= 20

        surge = msg.surge * self.speed/100
        sway = msg.sway * self.speed/100
        heave = msg.heave * self.speed/100
        pitch = msg.pitch * self.speed/100
        yaw = msg.yaw * self.speed/100
        roll = msg.roll * self.speed

        TFR = (surge - sway - heave - pitch + yaw - roll)
        TFL = (surge + sway - heave - pitch - yaw + roll)
        TBR = (surge - sway + heave - pitch - yaw - roll)
        TBL = (surge + sway + heave - pitch + yaw + roll)
        BFR = (surge - sway + heave + pitch + yaw + roll)
        BFL = (surge + sway + heave + pitch - yaw - roll)
        BBL = (surge - sway - heave + pitch - yaw + roll)
        BBR = (surge + sway - heave + pitch + yaw - roll)



        self.speeds = [TFR, TFL, TBR, TBL, BFR, BFL, BBR, BBL]
        self.limiting_current()
        self.backward_scaling()

        self.get_logger().info(f"{self.speeds}")

        
        return self.speeds
        
    def limiting_current(self):
        max_val = max(abs(s) for s in self.speeds)
        if max_val > self.max_power and max_val > 0.0:
            scale = self.max_power / max_val
            self.speeds = [s * scale for s in self.speeds]

    def backward_scaling(self):
        # (lower offset, upper offset, factor): lower bound exclusive, upper inclusive
        segments = [
            (30,   45,  1),
            (45,   65,  0.95),
            (65,  100,  0.92),
            (100, 160,  0.89),
            (160, 200,  0.87),
        ]

        # Only scale when some thruster is reversing
        if not any(s < 1 for s in self.speeds):
            return

        for i, s in enumerate(self.speeds):
            for low, high, factor in segments:
                if low < s <= high:
                    self.speeds[i] = s * factor
                    break





def main(args=None):
    rclpy.init(args=args)
    node = Motion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# class ThrusterMotionNode(Node):
#     def __init__(self):
#         super().__init__('thruster_motion_node')

#         #defining parameters and constants
#         self.max_power = 0.85
#         self.deadband = 0.05
#         self.motor_dirs = [1.0]*8
#         self.pwm_min = 1262
#         self.pwm_mid = 1500
#         self.pwm_max = 1736
#         self.esc_deadband = 25

#         # Joystick axes: [forward, rotate, slide, pitch, up, roll]
#         self.joy_axes = [0.0] * 6

#         # Subscriptions
#         self.sub_joy = self.create_subscription(
#             Float32MultiArray, 'joystick_motion', self.joy_callback, 10
#         )

#         # Publishers
#         self.pub_serial = self.create_publisher(
#             Int16MultiArray, '/rov/serial', 10
#         )
#         self.pub_motion = self.create_publisher(
#             Float32MultiArray, '/rov/motion', 10
#         )

#         # 50 Hz control loop timer
#         self.timer = self.create_timer(0.02, self.update_thrusters)
#         self.get_logger().info('Thruster motion node started (PWM: 1262 - 1736).')

#     def joy_callback(self, msg):
#         for i in range(min(len(msg.data), 6)):
#             val = msg.data[i]
#             self.joy_axes[i] = val if abs(val) > self.deadband else 0.0

#     def update_thrusters(self):
#         # 1. Read pilot commands directly
#         forward = self.joy_axes[0]
#         rotate  = self.joy_axes[1]
#         slide   = self.joy_axes[2]
#         pitch   = self.joy_axes[3]
#         up      = self.joy_axes[4]
#         roll    = self.joy_axes[5]

#         # 2. Simple mixing equations for 8 thrusters (unit weights)
#         # Top thrusters
#         tfr = +forward - slide - up - rotate - pitch - roll
#         tfl = +forward + slide - up + rotate - pitch + roll
#         tbr = +forward + slide - up + rotate + pitch - roll
#         tbl = +forward - slide - up - rotate + pitch + roll

#         # Bottom thrusters
#         bfr = +forward - slide + up - rotate - pitch + roll
#         bfl = +forward + slide + up + rotate - pitch - roll
#         bbr = +forward + slide + up + rotate + pitch + roll
#         bbl = +forward - slide + up - rotate + pitch - roll

#         speeds = [tfr, tfl, tbr, tbl, bfr, bfl, bbr, bbl]

#         #motor polarity inversion
#         speeds = [s * d for s, d in zip(speeds, self.motor_dirs)]

#         # Proportional de-saturation (48V / 15A protection)
#         max_val = max(abs(s) for s in speeds)
#         if max_val > self.max_power and max_val > 0.0:
#             scale = self.max_power / max_val
#             speeds = [s * scale for s in speeds]

#         # Convert to ESC PWM pulses (1262 to 1736 us, 1500 neutral)
#         forward_start = self.pwm_mid + self.esc_deadband  # 1525
#         reverse_start = self.pwm_mid - self.esc_deadband  # 1475

#         pwms = []
#         for s in speeds:
#             if abs(s) < 0.01:
#                 pwms.append(self.pwm_mid)
#             elif s > 0:
#                 pwms.append(int(forward_start + s * (self.pwm_max - forward_start)))
#             else:
#                 pwms.append(int(reverse_start + s * (reverse_start - self.pwm_min)))

#         # Publish PWMs to serial bridge node (Pico)
#         serial_msg = Int16MultiArray()
#         serial_msg.data = pwms
#         self.pub_serial.publish(serial_msg)

#         # Publish percentage throttle to GUI
#         motion_msg = Float32MultiArray()
#         motion_msg.data = [round(s * 100.0, 1) for s in speeds]
#         self.pub_motion.publish(motion_msg)

