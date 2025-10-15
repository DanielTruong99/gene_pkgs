from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    remote_controller = Node(
        package="remote_controller_cpp",
        executable="remote_controller_node",
        name="remote_controller",
    )

    joystick_driver = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
    )

    delayed_remote_controller = TimerAction(
        period=3.0,
        actions=[remote_controller],
    )

    return LaunchDescription([joystick_driver, delayed_remote_controller])
