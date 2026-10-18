import rclpy
from rclpy.node import Node
from rov_interfaces.msg import ObjectData, AutonomousCommand, RovCommand
from collections import deque
from sensor_msgs.msg import Imu
from math import pi as PI
import numpy as np


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
        self.window_size = 10
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
            cmd = self.stop_moving(stability_stage=1)
        
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
            cmd = self.stop_moving(stability_stage=2)

        elif self.is_stable_2 and not self.at_original_pitch:
            cmd = self.move_to_original_pitch()

        elif self.at_original_pitch and not self.is_stable_3:
            cmd = self.stop_moving(stability_stage=3)

        elif self.is_stable_3 and not self.moved_boosting_step:
            cmd = self.move_boosting_step()

        elif self.moved_boosting_step and not self.is_stable_4:
            cmd = self.stop_moving(stability_stage=4)

        elif self.is_stable_4 and not self.finished:
            cmd = self.let_go()


        self.command_pub.publish(cmd)

    def start_search(self):
        cmd = AutonomousCommand()
        
        # 1. Check pressure/depth from IMU data
        # Define max_search_depth threshold according to your pool setup
        max_search_depth = 0.5  # e.g., 0.5 meters from the surface
        
        if self.imu_readings is not None and hasattr(self.imu_readings, 'depth'):
            if self.imu_readings.depth <= max_search_depth:
                self.reached_to_max_height = True

        # 2. Execution Logic based on altitude limit
        if not self.reached_to_max_height:
            # Slow ascent + slow yaw rotation
            cmd.linear_z = 15   # Controlled vertical velocity
            cmd.angular_z = 12  # Slow yaw to prevent frame blur and overshooting
        else:
            self.get_logger().info('Reached max altitude limit. Holding depth and scanning...')
            # Stop ascending, hold current depth, keep scanning in place
            cmd.linear_z = 0
            cmd.angular_z = 12

        # 3. Explicitly zero unhandled command fields for motor safety
        cmd.linear_x = 0
        cmd.linear_y = 0
        cmd.angular_x = 0
        cmd.angular_y = 0
        cmd.gripper = 0

        return cmd


    def stop_moving(self, stability_stage: int) -> AutonomousCommand:
        """
        Commands zero velocity setpoints to let low-level controllers hold position,
        and calculates roll/pitch variance over a 1-second rolling window (10 Hz).
        When variance drops below threshold, sets the corresponding stability flag.
        """
        cmd = AutonomousCommand()
        
        # Explicit zero commands for motion team PID lock
        cmd.linear_x = 0
        cmd.linear_y = 0
        cmd.linear_z = 0
        cmd.angular_x = 0
        cmd.angular_y = 0
        cmd.angular_z = 0

        # Verify IMU data availability
        if self.imu_readings is not None:
            current_roll = self.imu_readings.orientation.x
            current_pitch = self.imu_readings.orientation.y

            # Combined tilt magnitude Jitter
            tilt_magnitude = (current_roll ** 2 + current_pitch ** 2) ** 0.5
            self.error_buffer.append(tilt_magnitude)

            # Check variance once the 1-second window is populated
            if len(self.error_buffer) == self.window_size:
                mean = sum(self.error_buffer) / self.window_size
                variance = sum((x - mean) ** 2 for x in self.error_buffer) / self.window_size

                # Jitter settled below threshold
                if variance < self.stability_threshold:
                    self.get_logger().info(f'ROV stabilized at stage {stability_stage}! (Variance: {variance:.4f})')
                    
                    if stability_stage == 1:
                        self.is_stable_1 = True
                    elif stability_stage == 2:
                        self.is_stable_2 = True
                    elif stability_stage == 3:
                        self.is_stable_3 = True
                    elif stability_stage == 4:
                        self.is_stable_4 = True

                    self.error_buffer.clear()

        return cmd


    def move_to_oak_camera_region(self, camera, x, y):
        cmd = AutonomousCommand()

        if camera == 2:
            cmd.movements_direction.angular.z = (x-1920)/1920*180
            cmd.movements_direction.angular.y = (y-1080)/1080*180
        
        elif camera == 3:
            cmd.movements_direction.angular.z = (x+1920)/1920*180
            cmd.movements_direction.angular.y = (y-1080)/1080*180

        elif camera == 4:
            cmd.movements_direction.angular.z = (x+1920)/1920*180
            cmd.movements_direction.angular.y = (y+1080)/1080*180

        elif camera == 5:
            cmd.movements_direction.angular.z = (x-1920)/1920*180
            cmd.movements_direction.angular.y = (y+1080)/1080*180

    def move_to_oak_camera_center(self, camera, x, y):
        
        cmd = AutonomousCommand()

        cmd.movements_direction.angular.z = (x-1920/2)/1920*180
        cmd.movements_direction.angular.y = (y-1080/2)/1080*180

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