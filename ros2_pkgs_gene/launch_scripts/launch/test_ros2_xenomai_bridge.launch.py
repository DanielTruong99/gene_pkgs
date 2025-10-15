from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define the path to the other launch files
    ros2_xenomai_bridge_pkg_share = get_package_share_directory('ros2_xenomai_bridge')
    ros2_xenomai_bridge_launch = os.path.join(ros2_xenomai_bridge_pkg_share, 'launch', 'ros2_xenomai_bridge.launch.py')

    return LaunchDescription([
        # Include the ros2_xenomai_bridge_launch launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ros2_xenomai_bridge_launch)
        ),
        
        # Optionally, add more nodes or actions here
    ])