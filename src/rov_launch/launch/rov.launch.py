from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller',
            executable='joystick',
            name='joystick',
            output='screen',
        ),
        Node(
            package='rov_actuators',
            executable='actuators',
            name='actutators',
            output='screen',
        ),
        Node(
            package='rov_motion',
            executable='motion',
            name='motion_node',
            output='screen',
        ),
        Node(
            package='autonomous_control',
            executable='autonomous_control',
            name='autonomous_control_node',
            output='screen',
        ),
        
        
    ])