"""Fuse Nano accelerometer and gyroscope samples into ROV attitude."""

import math
import time

import numpy as np
import rclpy
from ahrs.common.orientation import q2euler
from ahrs.filters import EKF
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray


def quaternion_from_acceleration(accel_g: np.ndarray) -> np.ndarray:
    """Initialize roll and pitch from gravity; yaw is unobservable without a magnetometer."""
    ax, ay, az = accel_g
    roll = math.atan2(ay, az)
    pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
    half_roll = roll / 2.0
    half_pitch = pitch / 2.0
    return np.array(
        [
            math.cos(half_roll) * math.cos(half_pitch),
            math.sin(half_roll) * math.cos(half_pitch),
            math.cos(half_roll) * math.sin(half_pitch),
            -math.sin(half_roll) * math.sin(half_pitch),
        ],
        dtype=float,
    )


class AttitudeEKF(Node):
    """Subscribe to calibrated IMU samples and publish pitch, roll, yaw in degrees."""

    def __init__(self) -> None:
        super().__init__("rov_attitude_ekf")
        self.declare_parameter("yaw_enabled", False)
        self.declare_parameter("input_topic", "/rov/imu")
        self.declare_parameter("output_topic", "/rov/attitude")

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.publisher = self.create_publisher(
            Float32MultiArray, output_topic, qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            Float32MultiArray,
            input_topic,
            self.imu_callback,
            qos_profile_sensor_data,
        )

        self.ekf = EKF(frequency=100.0)
        self.quaternion: np.ndarray | None = None
        self.previous_sample_time: float | None = None
        self.last_warning_time = 0.0
        self.get_logger().info(
            f"EKF subscribed to {input_topic}; publishing [roll, pitch, yaw] "
            f"[pitch, roll, yaw] in degrees on {output_topic}. Yaw enabled: "
            f"{bool(self.get_parameter('yaw_enabled').value)}"
        )

    def imu_callback(self, message: Float32MultiArray) -> None:
        if len(message.data) != 7:
            self.warn_throttled("Expected /rov/imu layout [ax, ay, az, gx, gy, gz, temp]")
            return

        values = np.asarray(message.data, dtype=float)
        if not np.all(np.isfinite(values)):
            self.warn_throttled("Ignoring non-finite IMU sample")
            return

        accel_g = values[:3]
        gyro_rad_s = np.radians(values[3:6])
        accel_norm = float(np.linalg.norm(accel_g))
        if accel_norm < 1e-6:
            self.warn_throttled("Ignoring IMU sample with zero acceleration")
            return

        now = time.monotonic()
        if self.quaternion is None:
            self.quaternion = quaternion_from_acceleration(accel_g)
            self.previous_sample_time = now
            self.publish_attitude(self.quaternion)
            return

        assert self.previous_sample_time is not None
        dt = now - self.previous_sample_time
        self.previous_sample_time = now
        if dt <= 0.0:
            return
        dt = min(dt, 0.25)

        try:
            self.quaternion = self.ekf.update(
                q=self.quaternion,
                gyr=gyro_rad_s,
                acc=accel_g,
                dt=dt,
            )
        except (ValueError, np.linalg.LinAlgError) as error:
            self.warn_throttled(f"EKF update failed; reinitializing from gravity: {error}")
            self.quaternion = quaternion_from_acceleration(accel_g)
            return

        self.publish_attitude(self.quaternion)

    def publish_attitude(self, quaternion: np.ndarray) -> None:
        angles_deg = np.degrees(q2euler(quaternion))
        yaw_enabled = bool(self.get_parameter("yaw_enabled").value)
        output = Float32MultiArray()
        output.data = [
            float(angles_deg[1]),
            float(angles_deg[0]),
            float(angles_deg[2]) if yaw_enabled else 0.0,
        ]
        self.publisher.publis
        self.get_logger().warning("ARM requested")

    # ------------------------------------------------------- writing to Nano

    def send_command_packet(self) -> None:h(output)

    def warn_throttled(self, text: str) -> None:
        now = time.monotonic()
        if now - self.last_warning_time >= 2.0:
            self.get_logger().warning(text)
            self.last_warning_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AttitudeEKF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
