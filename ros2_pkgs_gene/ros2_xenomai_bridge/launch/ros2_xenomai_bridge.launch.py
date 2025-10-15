from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    motor_pub = Node(
        package='ros2_xenomai_bridge',
        executable='motor_state_pub',
        name='motor_driver',
        output='screen',
    )

    motor_sub = Node(
        package='ros2_xenomai_bridge',
        executable='motor_cmd_sub',
        name='motor_command_subscriber',
        output='screen',
    )

    return LaunchDescription([
        motor_pub,
        motor_sub
    ])