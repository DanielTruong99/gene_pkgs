from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define the path to the other launch files
    motor_driver_pkg_share = get_package_share_directory('motor_driver')
    motor_driver_launch = os.path.join(motor_driver_pkg_share, 'launch', 'motor_driver.launch.py')

    return LaunchDescription([
        # Include the motor driver launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(motor_driver_launch)
        ),
        
        # Optionally, add more nodes or actions here
    ])