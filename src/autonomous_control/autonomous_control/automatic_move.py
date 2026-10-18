import rclpy
from rclpy.node import Node
from rov_interfaces.msg import ObjectData, AutonomousCommand, RovCommand
from collections import deque
from sensor_msgs.msg import Imu
from math import pi as PI


class AutonomousControlNode(Node):
    def __init__(self):
        super().__init__('autonomous_control_node')

        # Finite State Machine Logic
        self.autonomous_mode = False
        self.search = False
        self.found = False
        self.is_stable_1 = False
        self.in_oak_camera_region = False
        self.in_oak_camera_center = False
        self.bounding_box_correct_size = False
        self.is_stable_2 = False
        self.at_original_pitch = False
        self.is_stable_3 = False
        self.moved_boosting_step = False
        self.is_stable_4 = False
        self.finished = False

        # Subscribers
        self.serial_sub = self.create_subscription(
            RovCommand,
            '/rov/serial',
            self.serial_callback,
            10
        )
        
        self.model_sub = self.create_subscription(
            ObjectData,
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

        # PID parameters for station keeping / stabilization in stop_moving
        self.kp = 2.0
        self.ki = 0.05
        self.kd = 0.5
        self.last_error = 0.0
        self.dt = 0.1

        # position check
        self.reached_to_max_height = False

        # Buffer to keep the last 50 IMU readings (~0.5 seconds at 100 Hz)
        self.window_size = 50
        self.error_buffer = deque(maxlen=self.window_size)

        # Thresholds
        self.stability_threshold = 0.05  # Maximum allowed standard deviation

        self.last_detection_of_target = None

        self.get_logger().info('Autonomous Control Node Started.')


    def serial_callback(self, msg: RovCommand):
        self.autonomous_mode = msg.autonomous_mode

    def imu_callback(self, msg: Imu):
        self.imu_readings = msg

    def model_callback(self, msg: ObjectData):

        # self.search = False
        # self.found = False
        # self.is_stable_1 = False
        # self.in_oak_camera_region = False
        # self.in_oak_camera_center = False
        # self.bounding_box_correct_size = False
        # self.is_stable_2 = False
        # self.at_original_pitch = False
        # self.is_stable_3 = False
        # self.moved_boosting_step = False
        # self.is_stable_4 = False
        # self.finished = False

        if not self.autonomous_mode:
            return

        cmd = AutonomousCommand()

        if msg.camera != 0: 
            self.last_detection_of_target = msg
            self.found = True

        if msg.camera == 0 and not self.found:
            self.get_logger().info('Searching...')
            cmd = self.start_search()

        elif self.found and not self.is_stable_1:
            self.get_logger().info('Positioning the ROV')
            cmd = self.stop_moving()
        
        elif self.is_stable_1 and not self.in_oak_camera_region:
            cmd = self.move_to_oak_camera_region(msg.camera, msg.x, msg.y)
        
        elif self.in_oak_camera_region and not self.in_oak_camera_center:
            cmd = self.move_to_oak_camera_center(msg.camera, msg.x, msg.y)

        elif self.in_oak_camera_center and not self.bounding_box_correct_size:
            cmd = self.move_forward(msg.height, msg.width)

            if msg.height > 100 and msg.width > 100:
                self.bounding_box_correct_size = True
                self.get_logger().info('Correct size')

        elif self.bounding_box_correct_size and not self.is_stable_2:
            cmd = self.stop_moving()

        elif self.is_stable_2 and not self.at_original_pitch:
            cmd = self.move_to_original_pitch()

        elif self.at_original_pitch and not self.is_stable_3:
            cmd = self.stop_moving()

        elif self.is_stable_3 and not self.moved_boosting_step:
            cmd = self.move_boosting_step()

        elif self.moved_boosting_step and not self.is_stable_4:
            cmd = self.stop_moving()

        elif self.is_stable_4 and not self.finished:
            cmd = self.let_go()


        self.command_pub.publish(cmd)

    def start_search(self):

        P = 10
        I = 10
        D = 10
        
        cmd = AutonomousCommand()
        
        if not self.reached_to_height:
            e = (self.initial_imu_readings.orientation.x + 1.0) - self.imu_readings.orientation.x
            if e < 0.1:
                self.reached_to_height = True
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

    def move_forward(self, height, width):

        cmd = AutonomousCommand()

        if not height in range(90, 110) or not width in range(90, 110):
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
        pass

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()