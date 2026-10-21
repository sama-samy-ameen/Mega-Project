import rclpy
from rclpy.node import Node
from rov_interfaces.msg import ActuatorsCommands


class Actuators(Node):
    def __init__(self):
        super().__init__('actuators')

        # Publish actuators commands
        self.publisher = self.create_publisher(ActuatorsCommands, '/actuators', 10)

        # Data publish rate (10 Hz)
        self.timer = self.create_timer(0.1, self.commands)

        # Subscriber taking data from joystick node
        self.serial_subscriber = self.create_subscription(
            ActuatorsCommands,
            '/joystick_actuators',
            self.actions,
            10
        )

        self.command = 0
        self.latest_msg = ActuatorsCommands()

    def actions(self, msg: ActuatorsCommands):
        # 0,1,2: pistons on gripper 1 | 3: piston on gripper 2 | 4,5: pistons | 6: motor
        if msg.p1:
            self.command |= (1 << 0)
        else:
            self.command &= ~(1 << 0)

        if msg.p2:
            self.command |= (1 << 1)
        else:
            self.command &= ~(1 << 1)

        if msg.p3:
            self.command |= (1 << 2)
        else:
            self.command &= ~(1 << 2)

        if msg.p4:
            self.command |= (1 << 3)
        else:
            self.command &= ~(1 << 3)

        # Fixed indentation (previously nested inside p4's else block)
        if msg.p5:
            self.command |= (1 << 4)
        else:
            self.command &= ~(1 << 4)

        if msg.p6:
            self.command |= (1 << 5)
        else:
            self.command &= ~(1 << 5)

        if msg.motor:
            self.command |= (1 << 6)
        else:
            self.command &= ~(1 << 6)

        # Store updated message for the timer to publish cleanly
        msg.command = self.command
        self.latest_msg = msg

    def commands(self):
        # Publishes the full state (both booleans and bitmask command) at 10 Hz
        self.latest_msg.command = self.command
        self.publisher.publish(self.latest_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Actuators()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()