from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('joint_planner'),
        'config',
        'robot_controller_controlledyaw_latest_2.yaml'
    )

    node = Node(
        package='joint_planner',
        executable='joint_planner',
        name='joint_planner',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([
        node
    ])