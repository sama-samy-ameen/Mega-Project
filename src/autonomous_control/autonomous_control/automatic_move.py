import rclpy
from rclpy.node import Node
from rov_interfaces.msg import ModelReadings, AutonomousCommand

class AutonomousControlNode(Node):
    def __init__(self):
        super().__init__('autonomous_control_node')

        # Subscriber to sensor telemetry
        self.model_readings = self.create_subscription(
            ModelReadings,
            '/model_readings',
            self.model_callback,
            10
        )

        # Publisher for autonomous commands
        self.command_pub = self.create_publisher(
            AutonomousCommand,
            '/autonomous_command',
            10
        )

        self.get_logger().info('Autonomous Control Node Started.')

    def model_readings(self, msg: ModelReadings):
        self.get_logger().info(f'Current Depth: {msg.depth}m')

        # Create and publish autonomous command
        cmd = AutonomousCommand()
        # cmd.

        self.command_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()