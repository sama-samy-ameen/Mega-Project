import rclpy
from rclpy.node import Node
from rov_interfaces.msg import MotionCommands, ThrustersSpeeds
from std_msgs.msg import Float32MultiArray


class Motion(Node):
    def __init__(self):
        super().__init__("motion_node")
        self.speed = 20
        self.max_power = 200
        self.speeds = []
        self.depth_hold_active=False
        self.target_depth=0.0
        self.previous_depth_hold_button=False
        self.current_depth=None
        self.last_depth_time=None
        self.create_subscription(MotionCommands, "/joystick_motion", self.joystick_motion, 10)
        #create subscription for depth 
        self.create_subscription(Float32MultiArray, "/rov/serial", self.depth_callback, 10)
        self.speed_pub = self.create_publisher(ThrustersSpeeds, "/rov/motion", 10)


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

        #simple depth hold logic
        #to do : fix overshooting issue here
        #Toggle Depth Hold whenever R3 is clicked
        if msg.depth_hold and not self.previous_depth_hold_button:
            if not self.depth_hold_active:
                if self.current_depth is not None:
                    self.target_depth=self.current_depth
                    self.depth_hold_active=True
                    self.get_logger().info(f"==> Depth Hold ON: Locked at {self.target_depth:.2f} m")

                else:
                    self.get_logger().warning("Cannot enable depth hold: no depth reading recieved")

            else:
                self.depth_hold_active= False
                self.get_logger().info("==> Depth Hold OFF: Manual mode")

        self.previous_depth_hold_button=msg.depth_hold

        if self.depth_hold_active and self.current_depth is not None:
            error = self.current_depth - self.target_depth
            kp = 50.0  # Initial value to tune
            correction = kp * error

            correction = max(-40.0, min(40.0, correction))
            heave = correction

        self.eight_thrust_configuration(surge, sway, heave, pitch, yaw, roll)
        # self.seven_thrust_configuration(surge, sway, heave, pitch, yaw, roll)
        # self.six_thrust_configuration(surge, sway, heave, pitch, yaw, roll)

        self.limiting_current()
        self.backward_scaling()
        for i, s in enumerate(self.speeds):
            self.speeds[i] = int(s)

        self.speed_publish()
        

    def eight_thrust_configuration(self, surge, sway, heave, pitch, yaw, roll):
        TFR = (surge - sway - heave - pitch + yaw - roll)
        TFL = (surge + sway - heave - pitch - yaw + roll)
        TBR = (surge - sway + heave - pitch - yaw - roll)
        TBL = (surge + sway + heave - pitch + yaw + roll)
        BFR = (surge - sway + heave + pitch + yaw + roll)
        BFL = (surge + sway + heave + pitch - yaw - roll)
        BBL = (surge - sway - heave + pitch - yaw + roll)
        BBR = (surge + sway - heave + pitch + yaw - roll)

        self.speeds = [TFR, TFL, TBR, TBL, BFR, BFL, BBR, BBL]

    def seven_thrust_configuration(self, surge, sway, heave, pitch, yaw, roll):
        TFR = (-heave + yaw + roll)
        TFL = (-heave)
        TBR = (surge - sway + heave - pitch + roll)
        TBL = (surge - sway + heave - pitch - yaw)
        BFR = (surge - sway + pitch + yaw - roll)
        BFL = (surge - sway + pitch)
        BBL = (-yaw + roll)
        BBR = 0

        self.speeds = [TFR, TFL, TBR, TBL, BFR, BFL, BBR, BBL]

    def six_thrust_configuration(self, surge, heave, pitch, yaw):
        TFR = (surge - heave + yaw)
        TFL = (surge - heave - yaw)
        TBR = (heave - pitch + yaw)
        TBL = (heave - pitch - yaw)
        BFR = (surge + pitch)
        BFL = (surge + pitch)
        BBR = 0
        BBL = 0

        self.speeds = [TFR, TFL, TBR, TBL, BFR, BFL, BBR, BBL]

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

    def depth_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self.current_depth = msg.data[2]
            self.last_depth_time = self.get_clock().now()

    def speed_publish(self):
        msg = ThrustersSpeeds()
        msg.tfr = self.speeds[0]
        msg.tfl = self.speeds[1]
        msg.tbr = self.speeds[2]
        msg.tbl = self.speeds[3]
        msg.bfr = self.speeds[4]
        msg.bfl = self.speeds[5]
        msg.bbr = self.speeds[6]
        msg.bbl = self.speeds[7]

        self.speed_pub.publish(msg)




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

