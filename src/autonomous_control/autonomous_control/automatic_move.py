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

        # Internal PID Error Accumulators
        self.last_error_x = 0.0
        self.last_error_y = 0.0
        self.last_error_forward = 0.0
        self.last_error_pitch = 0.0

        # IMU Fallback Values (since imu_callback is removed)
        self.current_roll = 0.0
        self.current_pitch = 0.0

        # Boost tick state initialization
        self.boost_ticks = 0
        self.max_boost_ticks = 5

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
        
        elif self.is_stable_1 and not self.in_oak_camera_center:
            cmd = self.move_to_oak_camera_center(msg.camera, msg.x, msg.y)

        elif self.in_oak_camera_center and not self.bounding_box_correct_size:
            cmd = self.move_forward(msg.height, msg.width)

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

        if self.is_stable_4 and not self.finished:
            cmd = self.let_go()

        cmd.autonomous_mode = self.autonomous_mode
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
                std_dev = variance ** 0.5

                # Jitter settled below threshold
                if std_dev < self.stability_threshold:
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


    def move_to_oak_camera_center(self, camera: int, x: int, y: int) -> AutonomousCommand:
        cmd = AutonomousCommand()

        # 1. Target Data Fallback (Handling Camera Frame Loss)
        if camera == 0:
            if self.last_detection_of_target is not None:
                # Use cached pixel offsets from the last valid vision frame
                x = self.last_detection_of_target.x
                y = self.last_detection_of_target.y
            else:
                # No vision data available yet; hold current position
                return cmd

        # Frame center setup (assuming 1920x1080 resolution)
        x_center = 1920 / 2.0
        y_center = 1080 / 2.0
        deadband = 40  # Pixel tolerance radius

        # Compute Pixel Errors
        error_x = x - x_center
        error_y = y_center - y  # Inverted so positive error = need to go UP

        # 2. Check if Target is Centered
        if abs(error_x) <= deadband and abs(error_y) <= deadband:
            self.in_oak_camera_center = True
            self.get_logger().info('Target locked in OAK-D Center!')
            cmd.angular_z = 0
            cmd.linear_z = 0
            # Reset internal error accumulators for the next step
            self.last_error_x = 0.0
            self.last_error_y = 0.0
            return cmd

        # 3. Discrete PID Calculation for Yaw (Angular Z)
        # P-Term
        p_out_x = self.kp * error_x
        # D-Term (derivative of error over time step self.dt)
        d_out_x = self.kd * ((error_x - getattr(self, 'last_error_x', 0.0)) / self.dt)
        # Save last error
        self.last_error_x = error_x

        # 4. Discrete PID Calculation for Altitude (Linear Z)
        p_out_y = self.kp * error_y
        d_out_y = self.kd * ((error_y - getattr(self, 'last_error_y', 0.0)) / self.dt)
        self.last_error_y = error_y

        # 5. Output Clamp (-100 to +100 Max Relative Power)
        raw_yaw = p_out_x + d_out_x
        raw_depth = p_out_y + d_out_y

        cmd.angular_z = int(max(-50, min(50, raw_yaw)))    # Clamped to half-power max for stability
        cmd.linear_z = int(max(-40, min(40, raw_depth)))

        return cmd

    
    def move_forward(self, height: float, width: float) -> AutonomousCommand:
        cmd = AutonomousCommand()

        # 1. Fallback for Frame Loss
        if height == 0 or width == 0:
            if self.last_detection_of_target is not None:
                height = float(self.last_detection_of_target.height)
                width = float(self.last_detection_of_target.width)
            else:
                return cmd

        target_dimension = 100.0  # Desired pixel size for target distance
        tolerance = 5.0           # Acceptable pixel window (95 to 105 px)

        # 2. Angle Compensation: Use max dimension to prevent over-approaching due to perspective compression
        perceived_size = max(height, width)

        # Compute Error
        # Positive error = target is too far (ROV needs to move FORWARD)
        # Negative error = target is too close (ROV needs to move BACKWARD)
        error = target_dimension - perceived_size

        # 3. Check Target Arrival Window
        if abs(error) <= tolerance:
            self.bounding_box_correct_size = True
            self.get_logger().info('Target bounding box size reached!')
            cmd.linear_x = 0
            self.last_error_forward = 0.0
            return cmd

        # 4. Discrete PID Calculation for Forward Surge (Linear X)
        p_out = self.kp * error
        d_out = self.kd * ((error - getattr(self, 'last_error_forward', 0.0)) / self.dt)
        self.last_error_forward = error

        # 5. Command Output Clamping (-30 to +30 max surge)
        raw_surge = p_out + d_out
        cmd.linear_x = int(max(-30, min(30, raw_surge)))

        # Zero out unhandled axes
        cmd.linear_y = 0
        cmd.linear_z = 0
        cmd.angular_x = 0
        cmd.angular_y = 0
        cmd.angular_z = 0

        return cmd

    def move_to_original_pitch(self) -> AutonomousCommand:
        cmd = AutonomousCommand()

        if self.imu_readings is None:
            return cmd

        # Target pitch angle (0.0 degrees level)
        target_pitch = 0.0
        current_pitch = self.imu_readings.angular_y
        pitch_error = target_pitch - current_pitch
        pitch_tolerance = 1.0  # Allowed error margin in degrees

        # Check if pitch restoration is complete
        if abs(pitch_error) <= pitch_tolerance:
            self.at_original_pitch = True
            self.get_logger().info('Level pitch restored!')
            cmd.angular_y = 0
            self.last_error_pitch = 0.0
            return cmd

        # Discrete PID Calculation for Pitch (Angular Y)
        p_out = self.kp * pitch_error
        d_out = self.kd * ((pitch_error - self.last_error_pitch) / self.dt)
        self.last_error_pitch = pitch_error

        # Clamped pitch adjustment thrust
        raw_pitch_cmd = p_out + d_out
        cmd.angular_y = int(max(-25, min(25, raw_pitch_cmd)))

        return cmd

    def move_boosting_step(self) -> AutonomousCommand:
        cmd = AutonomousCommand()

        if not hasattr(self, 'boost_ticks'):
            self.boost_ticks = 0
            self.max_boost_ticks = 5  # ~0.5 seconds at 10 Hz

        if self.boost_ticks < self.max_boost_ticks:
            cmd.linear_x = 10  # Very gentle forward creep
            self.boost_ticks += 1
            self.get_logger().info(f'Nudging forward: {self.boost_ticks}/{self.max_boost_ticks}')
        else:
            cmd.linear_x = 0
            self.moved_boosting_step = True
            self.get_logger().info('Nudge complete!')
            delattr(self, 'boost_ticks')

        cmd.linear_y = 0
        cmd.linear_z = 0
        cmd.angular_x = 0
        cmd.angular_y = 0
        cmd.angular_z = 0

        return cmd

    def let_go(self) -> AutonomousCommand:
        cmd = AutonomousCommand()

        # Zero out all movement axes for total stationary release
        cmd.linear_x = 0
        cmd.linear_y = 0
        cmd.linear_z = 0
        cmd.angular_x = 0
        cmd.angular_y = 0
        cmd.angular_z = 0

        # Trigger payload release mechanism
        cmd.gripper = 0b00_00_00_00  # 1 = Release / Open command signal

        self.finished = True
        self.autonomous_mode = False
        self.get_logger().info('Coral transplantation payload released! Task finished.')

        return cmd

    def reset(self):
        cmd = AutonomousCommand()
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
        self.get_logger().info('Finished task.')
        self.get_logger().info('Resetting...')
        cmd.autonomous_mode = False
        return cmd

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()