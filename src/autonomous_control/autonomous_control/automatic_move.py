import rclpy
from rclpy.node import Node
from rov_interfaces.msg import ModelReadings, AutonomousCommand
from sensor_msgs.msg import Imu

PI = 3.14159265359

class AutonomousControlNode(Node):
    def __init__(self):
        super().__init__('autonomous_control_node')

        self.tasks = 0
        self.turns = 0
        self.held = False
        self.search = False
        self.found = False
        self.in_oak = False
        self.in_oak_camera_center = False
        self.correct_size = False
        self.in_gripper_camera_center = False

        # Subscriber to sensor telemetry
        self.model_readings = self.create_subscription(
            ModelReadings,
            '/model_readings',
            self.model_callback,
            10
        )

        self.imu_readings = self.create_subscription(
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

        self.get_logger().info('Autonomous Control Node Started.')

    def model_readings(self, msg: ModelReadings):

        if self.held:
            self.search = True
        
        self.found = False
        
        self.in_oak = False
        
        self.in_oak_camera_center = False
        
        self.correct_size = False
        
        self.in_gripper_camera_center = False

        # cmd = AutonomousCommand()
        

        self.command_pub.publish(cmd)

    def imu_callback(self, msg: Imu):
        pass

    def start_search(self):
        pass

    def move_to_oak_camera_center(self, camera, x, y):
        
        cmd = AutonomousCommand()

        if camera == 1:
            cmd.movements_direction.linear.x = (x-1920)/1920*2
            cmd.movements_direction.linear.y = (y-1080)/1080*2

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

    def move_forward(self):
        pass

    def move_to_gripper_camera_center(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()