from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rov_serial',
            executable='rov_serial_bridge',
            name='rov_serial_bridge',
            output='screen',
            ),

         Node(
            package='rov_serial',
            executable='rov_attitude_ekf',
            name='rov_attitude_ekf',
            output='screen',
            ),

        Node(
            package='RovGui',
            executable='rovgui',
            name='rov_dashboard',
            output='screen',
            ),
                
        Node(
            package='controller',
            executable='joystick',
            name='joystick',
            output='screen',
        ),
        Node(
            package='rov_motion',
            executable='motion',
            name='motion_node',
            output='screen',
            ),

        Node(
            package='rov_actuators',
            executable='actuators',
            name='actuators',
            output='screen',
        ),
        
        Node(
            package='crab_detection',
            executable='crab_detection_node',
            name='crab_detection_node',
            output='screen',
        ),

        Node(
            package='rov_vision',
            executable='red_square_node',
            name='red_square_node',
            output='screen',
            ),

        
    ])