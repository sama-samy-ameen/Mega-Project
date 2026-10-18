import rclpy
from rclpy.node import Node
from rov_interfaces.msg import ObjectData, AutonomousCommand, RovCommand
from collections import deque
from sensor_msgs.msg import Imu
import numpy as np


class AutonomousControlNode(Node):
    def __init__(self):
        super().__init__('autonomous_control_node')

        # Finite State Machine Flags
        self.autonomous_mode = False
        self.search = False
        self.found = False
        self.is_stable_1 = False
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

        # IMU & Sensor Placeholders (Edit these when custom IMU/Pressure msg arrives)
        self.imu_readings = None
        self.current_roll = 0.0      # Placeholder for roll angle
        self.current_pitch = 0.0     # Placeholder for pitch angle
        self.current_depth = 0.0     # Placeholder for pressure sensor depth reading

        # PID parameters (Scaled up for absolute speed units)
        self.kp = 4.0
        self.ki = 0.1
        self.kd = 1.0
        self.dt = 0.1

        # Internal PID Error Accumulators
        self.last_error_x = 0.0
        self.last_error_y = 0.0
        self.last_error_forward = 0.0
        self.last_error_pitch = 0.0

        # Boost tick state initialization
        self.boost_ticks = 0
        self.max_boost_ticks = 5

        # Position check
        self.reached_to_max_height = False

        # Rolling stability buffer
        self.window_size = 10
        self.error_buffer = deque(maxlen=self.window_size)
        self.stability_threshold = 0.05  # Maximum allowed standard deviation

        self.last_detection_of_target = None

        self.get_logger().info('Autonomous Control Node Started.')

    def serial_callback(self, msg: RovCommand):
        self.autonomous_mode = msg.autonomous_mode

    def imu_callback(self, msg: Imu):
        self.imu_readings = msg
        
        # --- CUSTOM SENSOR PLACEHOLDERS ---
        # Edit these assignments when motion team custom message fields are finalized:
        # self.current_roll = msg.roll
        # self.current_pitch = msg.pitch
        # self.current_depth = msg.depth
        pass

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
        
        elif self.is_stable_1 and not self.in_oak_camera_center:
            cmd = self.move_to_oak_camera_center(msg.camera, msg.x, msg.y)

        elif self.in_oak_camera_center and not self.bounding_box_correct_size:
            # Cast bounding box height/width to int16 as requested
            cmd = self.move_forward(int(msg.height), int(msg.width))

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

        # Always publish active autonomous mode status
        cmd.autonomous_mode = self.autonomous_mode
        self.command_pub.publish(cmd)

    def start_search(self) -> AutonomousCommand:
        cmd = AutonomousCommand()
        max_search_depth = 0.5  # meters from surface
        
        # Check depth using sensor placeholder
        if self.current_depth <= max_search_depth and self.imu_readings is not None:
            self.reached_to_max_height = True

        if not self.reached_to_max_height:
            # Scaled speed values (int16)
            cmd.linear_z = 30   # Vertical ascent speed unit
            cmd.angular_z = 25  # Yaw speed unit
        else:
            self.get_logger().info('Reached max altitude limit. Holding depth and scanning...')
            cmd.linear_z = 0
            cmd.angular_z = 25

        cmd.linear_x = 0
        cmd.linear_y = 0
        cmd.angular_x = 0
        cmd.angular_y = 0
        cmd.gripper = 0

        return cmd

    def stop_moving(self, stability_stage: int) -> AutonomousCommand:
        cmd = AutonomousCommand()
        
        cmd.linear_x = 0
        cmd.linear_y = 0
        cmd.linear_z = 0
        cmd.angular_x = 0
        cmd.angular_y = 0
        cmd.angular_z = 0

        # Variance calculation using IMU placeholders
        tilt_magnitude = (self.current_roll ** 2 + self.current_pitch ** 2) ** 0.5
        self.error_buffer.append(tilt_magnitude)

        if len(self.error_buffer) == self.window_size:
            mean = sum(self.error_buffer) / self.window_size
            variance = sum((x - mean) ** 2 for x in self.error_buffer) / self.window_size
            std_dev = variance ** 0.5

            if std_dev < self.stability_threshold:
                self.get_logger().info(f'ROV stabilized at stage {stability_stage}! (StdDev: {std_dev:.4f})')
                
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

    def move_to_oak_camera_center(self, camera: int, x: int, y: int) -> AutonomousCommand:
        cmd = AutonomousCommand()

        if camera == 0:
            if self.last_detection_of_target is not None:
                x = self.last_detection_of_target.x
                y = self.last_detection_of_target.y
            else:
                return cmd

        x_center = 1920 / 2.0
        y_center = 1080 / 2.0
        deadband = 40  

        error_x = x - x_center
        error_y = y_center - y  

        if abs(error_x) <= deadband and abs(error_y) <= deadband:
            self.in_oak_camera_center = True
            self.get_logger().info('Target locked in OAK-D Center!')
            cmd.angular_z = 0
            cmd.linear_z = 0
            self.last_error_x = 0.0
            self.last_error_y = 0.0
            return cmd

        # Discrete PID Calculation
        p_out_x = self.kp * error_x
        d_out_x = self.kd * ((error_x - self.last_error_x) / self.dt)
        self.last_error_x = error_x

        p_out_y = self.kp * error_y
        d_out_y = self.kd * ((error_y - self.last_error_y) / self.dt)
        self.last_error_y = error_y

        # Clamped output cast to int16 speed units (e.g., -100 to +100 max speed)
        cmd.angular_z = int(max(-100, min(100, p_out_x + d_out_x)))
        cmd.linear_z = int(max(-80, min(80, p_out_y + d_out_y)))

        return cmd

    def move_forward(self, height: int, width: int) -> AutonomousCommand:
        cmd = AutonomousCommand()

        if height == 0 or width == 0:
            if self.last_detection_of_target is not None:
                height = int(self.last_detection_of_target.height)
                width = int(self.last_detection_of_target.width)
            else:
                return cmd

        target_dimension = 100.0  
        tolerance = 5.0           

        perceived_size = max(height, width)
        error = target_dimension - perceived_size

        if abs(error) <= tolerance:
            self.bounding_box_correct_size = True
            self.get_logger().info('Target bounding box size reached!')
            cmd.linear_x = 0
            self.last_error_forward = 0.0
            return cmd

        p_out = self.kp * error
        d_out = self.kd * ((error - self.last_error_forward) / self.dt)
        self.last_error_forward = error

        # Clamped output cast to int16 speed units (e.g., -60 to +60 max speed)
        cmd.linear_x = int(max(-60, min(60, p_out + d_out)))

        cmd.linear_y = 0
        cmd.linear_z = 0
        cmd.angular_x = 0
        cmd.angular_y = 0
        cmd.angular_z = 0

        return cmd

    def move_to_original_pitch(self) -> AutonomousCommand:
        cmd = AutonomousCommand()

        target_pitch = 0.0
        pitch_error = target_pitch - self.current_pitch
        pitch_tolerance = 1.0  

        if abs(pitch_error) <= pitch_tolerance:
            self.at_original_pitch = True
            self.get_logger().info('Level pitch restored!')
            cmd.angular_y = 0
            self.last_error_pitch = 0.0
            return cmd

        p_out = self.kp * pitch_error
        d_out = self.kd * ((pitch_error - self.last_error_pitch) / self.dt)
        self.last_error_pitch = pitch_error

        # Clamped pitch speed unit (int16)
        cmd.angular_y = int(max(-50, min(50, p_out + d_out)))

        return cmd

    def move_boosting_step(self) -> AutonomousCommand:
        cmd = AutonomousCommand()

        if not hasattr(self, 'boost_ticks'):
            self.boost_ticks = 0
            self.max_boost_ticks = 5  

        if self.boost_ticks < self.max_boost_ticks:
            cmd.linear_x = 20  # Gentle forward surge speed unit (int16)
            self.boost_ticks += 1
            self.get_logger().info(f'Nudging forward: {self.boost_ticks}/{self.max_boost_ticks}')
        else:
            cmd.linear_x = 0
            self.moved_boosting_step = True
            self.get_logger().info('Nudge complete!')

        cmd.linear_y = 0
        cmd.linear_z = 0
        cmd.angular_x = 0
        cmd.angular_y = 0
        cmd.angular_z = 0

        return cmd

    def let_go(self) -> AutonomousCommand:
        cmd = AutonomousCommand()

        cmd.linear_x = 0
        cmd.linear_y = 0
        cmd.linear_z = 0
        cmd.angular_x = 0
        cmd.angular_y = 0
        cmd.angular_z = 0

        # Setting bitmask: 0 is OPEN per your team's protocol
        cmd.gripper = 0b00000000  

        self.finished = True
        self.autonomous_mode = False
        self.get_logger().info('Coral transplantation payload released! Task finished.')

        return cmd

    def reset(self):
        self.autonomous_mode = False
        self.search = False
        self.found = False
        self.is_stable_1 = False
        self.in_oak_camera_center = False
        self.bounding_box_correct_size = False
        self.is_stable_2 = False
        self.at_original_pitch = False
        self.is_stable_3 = False
        self.moved_boosting_step = False
        self.is_stable_4 = False
        self.finished = False
        self.boost_ticks = 0
        self.get_logger().info('Resetting autonomous state flags.')


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()