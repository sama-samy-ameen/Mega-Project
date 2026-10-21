"""ROS 2 serial bridge for thruster_controller.ino.

The firmware is the source of truth for the armed state. Whenever it reports
a disarm or a fault, or refuses an arm request, this node drops back to
disarmed instead of assuming it is still armed.
"""

import math
import struct
import threading
import time

import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32MultiArray


SYNC = b"\xA5\x5A"
HEARTBEAT_PERIOD_SECONDS = 0.05
NEUTRAL_PWM = 1500

# Firmware lines that mean "I refused to arm" or "something is wrong".
REFUSAL_LINES = (
    "FAULT,",
    "ERROR,IMU_NOT_READY",
    "ERROR,DISARM_FIRST",
    "ERROR,INVALID_ESC_PULSE",
    "ERROR,CALIBRATION_MODE",
)


def crc8(data: bytes) -> int:
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def build_command_packet(
    sequence: int, armed: bool, pwms: list[int], actuator_flags: int
) -> bytes:
    if len(pwms) != 8:
        raise ValueError("exactly eight thruster PWM values are required")
    if any(pulse < 1100 or pulse > 1900 for pulse in pwms):
        raise ValueError("thruster PWM values must be in the 1100-1900 us range")
    body = bytes((sequence & 0xFF, int(armed)))
    body += struct.pack("<8H", *pwms)
    body += bytes((actuator_flags & 0xFF,))
    return SYNC + body + bytes((crc8(body),))


class ROVSerialBridge(Node):
    def __init__(self) -> None:
        super().__init__("rov_serial_bridge")
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("motion_timeout_seconds", 0.5)
        self.serial_path = str(self.get_parameter("serial_port").value)
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.motion_timeout_seconds = float(
            self.get_parameter("motion_timeout_seconds").value
        )

        self.imu_publisher = self.create_publisher(Float32MultiArray, "/rov/imu", 10)
        self.depth_publisher = self.create_publisher(Float32, "/rov/depth", 10)
        self.armed_publisher = self.create_publisher(Bool, "/rov/armed", 10)
        self.create_subscription(Int32MultiArray, "/rov/motion", self.motion_callback, 10)
        self.create_subscription(Int32MultiArray, "/rov/actuators", self.states_callback, 10)
        self.create_subscription(Bool, "/rov/arm", self.arm_callback, 10)

        self.latest_pwms = [NEUTRAL_PWM] * 8
        self.actuator_flags = 0
        self.armed = False
        self.last_motion_time = 0.0
        self.sequence = 0
        self.firmware_ready = False
        self.running = True
        self.serial_device: serial.Serial | None = None
        self.write_lock = threading.Lock()
        self.last_serial_warning = 0.0

        self.create_timer(HEARTBEAT_PERIOD_SECONDS, self.send_command_packet)
        self.read_thread = threading.Thread(target=self.sensor_data, daemon=True)
        self.read_thread.start()
        self.get_logger().info(
            f"Bridge starting on {self.serial_path} at {self.baud_rate} baud; "
            "outputs remain disarmed until /rov/arm is true."
        )

    # ---------------------------------------------------------------- helpers

    def _warn_serial(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_serial_warning >= 2.0:
            self.get_logger().warning(message)
            self.last_serial_warning = now

    def force_disarm(self, reason: str) -> None:
        """Single place that returns everything to the safe state."""
        if self.armed:
            self.get_logger().warning(f"Disarmed: {reason}")
        self.armed = False
        self.latest_pwms = [NEUTRAL_PWM] * 8
        self.actuator_flags = 0

    def motion_is_fresh(self) -> bool:
        return (
            self.last_motion_time > 0.0
            and time.monotonic() - self.last_motion_time <= self.motion_timeout_seconds
        )

    # ------------------------------------------------------- reading from Nano

    def sensor_data(self) -> None:
        while self.running and rclpy.ok():
            device = self.serial_device
            if device is None or not device.is_open:
                self.open_serial()
                continue
            try:
                raw = device.readline()
            except (serial.SerialException, OSError) as error:
                self.close_serial(f"read failed: {error}")
                continue
            if raw.endswith(b"\n"):  # ignore lines cut off by the read timeout
                self.handle_line(raw.decode("ascii", errors="ignore").strip())

    def open_serial(self) -> None:
        self.firmware_ready = False
        try:
            self.serial_device = serial.Serial(
                self.serial_path, self.baud_rate, timeout=0.1, write_timeout=0.2
            )
            self.get_logger().info(f"Opened serial port {self.serial_path}")
        except serial.SerialException as error:
            self._warn_serial(f"Waiting for Nano serial port: {error}")
            time.sleep(1.0)

    def close_serial(self, reason: str) -> None:
        self._warn_serial(f"Serial {reason}; reconnecting")
        self.firmware_ready = False
        self.force_disarm("serial link lost")
        device, self.serial_device = self.serial_device, None
        if device is not None:
            try:
                device.close()
            except (serial.SerialException, OSError):
                pass
        time.sleep(0.25)

    def handle_line(self, line: str) -> None:
        if line.startswith("D,"):
            self.handle_imu(line)
        elif line.startswith("P,"):
            self.handle_pressure(line)
        elif line.startswith("ACK,"):
            self.handle_ack(line)
        elif line.startswith("STATE,DISARMED"):
            # Also matches "STATE,DISARMED,HOST_TIMEOUT" etc.
            if not self.firmware_ready:
                self.get_logger().info("Nano ready; heartbeat packets enabled")
            self.firmware_ready = True
            self.force_disarm(line)
        elif line.startswith(REFUSAL_LINES):
            self.get_logger().error(line)
            self.force_disarm(line)

    def handle_ack(self, line: str) -> None:
        # Format: ACK,<sequence>,<armed 0|1>
        parts = line.split(",")
        if len(parts) != 3:
            return
        self.firmware_ready = True
        self.armed_publisher.publish(Bool(data=(parts[2] == "1")))

    def handle_imu(self, line: str) -> None:
        fields = line.split(",")
        if len(fields) != 8:
            return
        try:
            values = [float(value) for value in fields[1:]]
        except ValueError:
            return
        if not all(math.isfinite(value) for value in values):
            return
        self.firmware_ready = True  # IMU data proves the Nano is up
        message = Float32MultiArray()
        message.data = values
        self.imu_publisher.publish(message)

    def handle_pressure(self, line: str) -> None:
        fields = line.split(",")
        if len(fields) != 2:
            return
        try:
            depth_m = float(fields[1])
        except ValueError:
            return
        if not math.isfinite(depth_m):
            return
        self.depth_publisher.publish(Float32(data=depth_m))

    # ---------------------------------------------------------- ROS callbacks

    def motion_callback(self, message: Int32MultiArray) -> None:
        if len(message.data) != 8:
            self.get_logger().warning("/rov/motion must contain exactly 8 PWM values")
            return
        if any(value < 1100 or value > 1900 for value in message.data):
            self.get_logger().warning("Rejected thruster PWM outside 1100-1900 us")
            return
        self.latest_pwms = [int(value) for value in message.data]
        self.last_motion_time = time.monotonic()

    def states_callback(self, message: Int32MultiArray) -> None:
        flags = 0
        for index, value in enumerate(message.data[:8]):
            if value:
                flags |= 1 << index
        self.actuator_flags = flags

    def arm_callback(self, message: Bool) -> None:
        if not message.data:
            self.force_disarm("operator disarm")
            self.get_logger().warning("DISARM requested")
            return
        if not self.motion_is_fresh():
            self.armed = False
            self.get_logger().warning("ARM rejected: no fresh /rov/motion command")
            return
        self.armed = True
        self.get_logger().warning("ARM requested")

    # ------------------------------------------------------- writing to Nano

    def send_command_packet(self) -> None:
        if self.armed and not self.motion_is_fresh():
            self.force_disarm("motion command timed out")

        device = self.serial_device
        if not self.firmware_ready or device is None or not device.is_open:
            return
        try:
            packet = build_command_packet(
                self.sequence, self.armed, self.latest_pwms, self.actuator_flags
            )
            with self.write_lock:
                device.write(packet)
            self.sequence = (self.sequence + 1) & 0xFF
        except (serial.SerialException, OSError, ValueError) as error:
            self._warn_serial(f"Serial write failed: {error}")
            self.force_disarm("serial write failed")

    def destroy_node(self) -> None:
        self.force_disarm("shutting down")
        self.firmware_ready = True
        for _ in range(3):  # a few copies in case one is lost
            self.send_command_packet()
            time.sleep(0.02)
        self.running = False
        if self.read_thread.is_alive():
            self.read_thread.join(timeout=0.5)
        device = self.serial_device
        if device is not None and device.is_open:
            try:
                device.flush()
                device.close()
            except (serial.SerialException, OSError):
                pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ROVSerialBridge()
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