import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import serial
import threading

BAUD_RATE = 115200 # we used higher baud rate to transmit data faster 

class ROVSerialBridge(Node):
    def __init__(self):
        super().__init__('rov_serial_bridge')

        # Connecting to Raspberry Pi Pico
        try:
            self.pico = serial.Serial('/dev/pts/2', BAUD_RATE, timeout=1)
            self.get_logger().info("Connected successfully to Raspberry Pi Pico serial bridge")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Pico: {e}")
            self.pico = None

        # Publisher for IMU and pressure sensor data
        self.publisher = self.create_publisher(Float32MultiArray, '/rov/serial', 10)

        # Subscriber for thruster PWM commands from high-level control
        self.motion_subscriber = self.create_subscription(Int32MultiArray, '/rov/motion',self.motion_callback,10)

        # Subscribes to actuator node for the 2 boolean states (pistons)
        self.actuator_subscription = self.create_subscription(Int32MultiArray, '/rov/actuators', self.states_callback, 10)

        # Cache variables to hold the latest states before sending a combined packet
        self.latest_pwms = [1500] * 8  # Default neutral stop for 8 thrusters
        self.boolean_flags = 0         # 2 states packed into a single byte via bitwise operators

        # Background thread to continuously read sensor data from the Pico
        self.running = True
        self.read_thread = threading.Thread(target=self.sensor_data)
        self.read_thread.daemon = True
        self.read_thread.start()

    def sensor_data(self):
        """Continuously reads telemetry lines from the Pico and publishes them."""
        while self.running and rclpy.ok():
            if self.pico and self.pico.is_open:
                try:
                    data = self.pico.readline().decode().strip()
                    if not data:
                        continue  # Skip empty lines

                    # Clean direct unpacking
                    roll, pitch, depth = data.split(',')

                    # Publish unified array: [roll, pitch, depth]
                    msg = Float32MultiArray()
                    msg.data = [float(roll), float(pitch), float(depth)]
                    self.publisher.publish(msg)
                    
                except Exception:
                    self.get_logger().warn('Error parsing sensor stream from Pico!')
                    self.pico.reset_input_buffer()

    def motion_callback(self, msg):
        """Receives 8 thruster PWM values from the motion node."""
        if len(msg.data) == 8:
            self.latest_pwms = list(msg.data)
            self.send_packet_to_pico()

    def states_callback(self, msg):
        """Receives multiple boolean states (pistons, relays, etc.) and packs them into 1 byte."""
        flags = 0
        
        # Loop through whatever boolean values are sent in the message (up to 8)
        for i, val in enumerate(msg.data):
            if i < 8:  # A single byte can hold a maximum of 8 boolean flags (bits 0-7)
                if bool(val):
                    flags |= (1 << i)  # Shift the 'i'th bit and turn it ON
                    
        self.boolean_flags = flags
        self.send_packet_to_pico()

    def send_packet_to_pico(self):
        """Assembles the 9-byte packet and sends it down to the Pico."""
        if not self.pico or not self.pico.is_open:
            return

        try:
            # Map 8 thruster PWMs (1100-1900) to bytes (0-255)
            thruster_bytes = bytes([
                int((max(1100, min(1900, val)) - 1100) * 255 / 800) 
                for val in self.latest_pwms
            ])
            
            # 9th byte is our packed bitwise flags byte
            flags_byte = bytes([self.boolean_flags])
            
            # Combine into a single 9-byte payload
            packet = thruster_bytes + flags_byte
            self.pico.write(packet)
        except Exception as e:
            self.get_logger().warn(f'Error sending packet to Pico: {e}')

    def destroy_node(self):
        self.running = False
        if self.pico and self.pico.is_open:
            self.pico.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ROVSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()