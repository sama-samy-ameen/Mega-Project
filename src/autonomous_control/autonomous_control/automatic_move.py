import rclpy
from rclpy.node import Node
from rov_interfaces.msg import ModelReadings, AutonomousCommand
from collections import deque
from sensor_msgs.msg import Imu
import numpy as np

PI = 3.14159265359

class AutonomousControlNode(Node):
    def __init__(self):
        super().__init__('autonomous_control_node')

        # Finite State Machine Logic
        self.tasks = 0
        self.turns = 0
        self.held = False
        self.search = False
        self.found = False
        self.in_oak = False
        self.in_oak_camera_center = False
        self.correct_size = False
        self.in_gripper_camera_center = False

        # Subscribers
        self.model_sub = self.create_subscription(
            ModelReadings,
            '/model_readings',
            self.model_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu_control',
            self.imu_callback,
            10
        )

        # Publisher for autonomous commands
        self.command_pub = self.create_publisher(
            AutonomousCommand,
            '/autonomous_command',
            10
        )

        # imu sitting
        self.imu_readings = None
        self.initial_imu_readings = None
        self.final_pos = None
        self.last_error = 0

        # PID parameters for station keeping / stabilization in stop_moving
        self.kp_stop = 2.0
        self.ki_stop = 0.05
        self.kd_stop = 0.5
        self.stop_integral = 0.0
        self.stop_last_error = 0.0
        self.stop_last_time = None

        # position check
        self.reached_to_hight = False
        self.done_full_rotation = False

        # Buffer to keep the last 50 IMU readings (~0.5 seconds at 100 Hz)
        self.window_size = 50
        self.error_buffer = deque(maxlen=self.window_size)

        # Thresholds
        self.stability_threshold = 0.05  # Maximum allowed standard deviation
        self.is_stable = False

        self.get_logger().info('Autonomous Control Node Started.')


    # def imu_callback(self, msg: Imu):

    def model_readings(self, msg: ModelReadings):

        if self.held and not self.search:
            self.search = True
            self.initial_imu_readings = self.imu_readings
            self.get_logger().info('Searching...')
            cmd = self.start_search()

        if self.tasks == 0 and self.search:
            if msg.targ_1.camera == 0:
                self.get_logger().info('Searching...')
                cmd = self.start_search()
            if not self.found:
                self.final_pos = self.imu_readings
                self.found = True
                self.get_logger().info('Location Detected')
                self.get_logger().info('Positioning the ROV')
                cmd = self.stop_moving()

        if self.is_stable and not self.in_oak_camera_center:
            cmd = self.move_to_oak_camera_center(msg.targ_1.camera, msg.targ_1.x, msg.targ_1.y)
        
        if self.in_oak_camera_center and not self.correct_size:
            cmd = self.move_forward(msg.targ_1.hight, msg.targ_1.width)

        if self.correct_size and not self.in_gripper_camera_center:
            cmd = self.move_to_gripper_camera_center(msg.targ_1.camera, msg.targ_1.x, msg.targ_1.y)

        if self.in_gripper_camera_center:
            cmd = self.let_go()
        

        self.command_pub.publish(cmd)

    def imu_callback(self, msg: Imu):
        self.imu_readings = msg

    def start_search(self):

        P = 10
        I = 10
        D = 10
        
        cmd = AutonomousCommand()
        
        if not self.reached_to_hight:
            e = (self.initial_imu_readings.orientation.x + 1.0) - self.imu_readings.orientation.x
            if e < 0.1:
                self.reached_to_hight = True
                cmd.movements_direction.linear.z = 0
                return cmd
            cmd.movements_direction.linear.z = P*e + I*self.last_error + D*(e - self.last_error)
            self.last_error = e
            return cmd

        
        if not self.done_full_rotation:
            e = self.initial_imu_readings.orientation.z - self.imu_readings.orientation.z
            if e < 0.1:
                self.done_full_rotation = True
                cmd.movements_direction.angular.z = 0
                return cmd
            cmd.movements_direction.angular.z = P*e + I*self.last_error + D*(e - self.last_error)
            self.last_error = e
            return cmd

    def stop_moving(self):

        cmd = AutonomousCommand()
        
        # Default all thruster outputs to zero
        cmd.movements_direction.linear.x = 0.0
        cmd.movements_direction.linear.y = 0.0
        cmd.movements_direction.linear.z = 0.0
        cmd.movements_direction.angular.x = 0.0
        cmd.movements_direction.angular.y = 0.0
        cmd.movements_direction.angular.z = 0.0

        if self.imu_readings is None:
            return cmd

        # 1. Compute your current IMU error (e.g., orientation or acceleration error)
        current_error = self.imu_readings.linear_acceleration.z - 9.81  # Example target

        # 2. Add current error to non-blocking rolling buffer
        self.error_buffer.append(current_error)

        # 3. Compute time delta (dt) for discrete PID
        current_time = self.get_clock().now().nanoseconds / 1e9
        if self.stop_last_time is None:
            dt = 0.01
        else:
            dt = current_time - self.stop_last_time
        self.stop_last_time = current_time

        if dt <= 0:
            dt = 0.01

        # 4. Evaluate stability when buffer fills up
        if len(self.error_buffer) == self.window_size:
            std_dev = np.std(self.error_buffer)
            self.is_stable = std_dev < self.stability_threshold

            if self.is_stable:
                self.get_logger().info(f"IMU Stable | Std Dev: {std_dev:.4f}")
                
                # Active PID counter-thrust to maintain steady position/leveling when stable
                self.stop_integral += current_error * dt
                derivative = (current_error - self.stop_last_error) / dt
                
                pid_output = (self.kp_stop * current_error) + (self.ki_stop * self.stop_integral) + (self.kd_stop * derivative)
                self.stop_last_error = current_error

                # Apply counter active effort to vertical stabilization
                cmd.movements_direction.linear.z = pid_output
                return cmd
            else:
                self.get_logger().warn(f"IMU Unstable (High Fluctuation) | Std Dev: {std_dev:.4f}")
                # Reset PID state on high noise/instability
                self.stop_integral = 0.0
                self.stop_last_error = 0.0
                return cmd

        return cmd


    def move_to_oak_camera_center(self, camera, x, y):
        
        cmd = AutonomousCommand()

        if camera == 1:
            cmd.movements_direction.angular.z = (x-1920/2)/1920*180
            cmd.movements_direction.angular.y = (y-1080/2)/1080*180

        elif camera == 2:
            cmd.movements_direction.angular.z = (x-1920)/1920*180
            cmd.movements_direction.angular.y = (y-1080)/1080*180
        
        elif camera == 3:
            cmd.movements_direction.angular.z = (x+1920)/1920*180
            cmd.movements_direction.angular.y = (y+1080)/1080*180
        
        # elif camera == 4:
        #     cmd.movements_direction.angular.z = (x-1920)/1920*180
        #     cmd.movements_direction.angular.y = (y-1080)/1080*180
        # elif camera == 5:
        #     cmd.movements_direction.angular.z = (x-1920)/1920*180
        #     cmd.movements_direction.angular.y = (y-1080)/1080*180

        return cmd

    def move_forward(self, hight, width):

        cmd = AutonomousCommand()

        if not hight in range(90, 110) or not width in range(90, 110):
            cmd.movements_direction.linear.x = 10

        return cmd

    def move_to_gripper_camera_center(self, camera, x, y):

        cmd = AutonomousCommand()
        
        if camera == 1:
            cmd.movements_direction.angular.z = (x+1920)/1920*180
            cmd.movements_direction.angular.y = (y+1080)/1080*180

        elif camera == 2:
            cmd.movements_direction.angular.z = (x-1920)/1920*180
            cmd.movements_direction.angular.y = (y-1080)/1080*180
        
        elif camera == 3:
            cmd.movements_direction.angular.z = (x+1920)/1920*180
            cmd.movements_direction.angular.y = (y+1080)/1080*180
        
        # elif camera == 4:
        #     cmd.movements_direction.angular.z = (x-1920)/1920*180
        #     cmd.movements_direction.angular.y = (y-1080)/1080*180
        # elif camera == 5:
        #     cmd.movements_direction.angular.z = (x-1920)/1920*180
        #     cmd.movements_direction.angular.y = (y-1080)/1080*180

        return cmd

    def let_go(self):
        cmd = AutonomousCommand()
        cmd.gripper = int(0b00_00_00_00)
        self.reset()
        return cmd

    def reset(self):
        self.held = False
        self.search = False
        self.found = False
        self.in_oak = False
        self.in_oak_camera_center = False
        self.correct_size = False
        self.in_gripper_camera_center = False
        self.stability_threshold = 0.05  # Maximum allowed standard deviation
        self.is_stable = False
        self.reached_to_hight = False
        self.done_full_rotation = False
        self.imu_readings = None
        self.initial_imu_readings = None
        self.final_pos = None
        self.last_error = 0
        self.stop_integral = 0.0
        self.stop_last_error = 0.0
        self.stop_last_time = None
        

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()