from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define the path to the other launch files
    joint_planner_pkg_share = get_package_share_directory('joint_planner')
    joint_planner_launch = os.path.join(joint_planner_pkg_share, 'launch', 'joint_planner.launch.py')

    # Define the path to the ros2_xenomai_bridge launch file
    ros2_xenomai_bridge_pkg_share = get_package_share_directory('ros2_xenomai_bridge')
    ros2_xenomai_bridge_launch = os.path.join(ros2_xenomai_bridge_pkg_share, 'launch', 'ros2_xenomai_bridge.launch.py')

    #define the path to vector nav launch file
    vector_nav_pkg_share = get_package_share_directory('vectornav')
    vector_nav_launch = os.path.join(vector_nav_pkg_share, 'launch', 'vectornav.launch.py')
    launch_vector_nav = IncludeLaunchDescription(PythonLaunchDescriptionSource(vector_nav_launch))

    #define the remote controller launch file
    remote_controller_pkg_share = get_package_share_directory('remote_controller_cpp')
    remote_controller_launch = os.path.join(remote_controller_pkg_share, 'launch', 'remote_controller.launch.py')

    delayed_bridge = TimerAction(
        period=3.0,  # seconds to wait before starting the bridge
        actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(ros2_xenomai_bridge_launch)),
            TimerAction(
                period=0.1,  # small gap to ensure ordering
                actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(joint_planner_launch))]
            )
        ]
    )

    # ros2 bag
    topics = ['/filtered_joint_cmd', '/joint_states', 'vectornav/raw/common']
    rosbag2 = ExecuteProcess(
        cmd=['ros2', 'bag', 'record'] + topics + ['--max-bag-size', '1073741824'],
        output='screen'
    )


    return LaunchDescription([
        # Include the motor driver launch file
        # ExecuteProcess(
        #     cmd=['roslaunch', 'EtherCAT_test', 'latest_motor_driver.launch'],
        #     output='screen'
        # ),

        launch_vector_nav,
        # delayed_bridge,

        rosbag2,
        # remote_controller_launch,
        
        # # Include the joint planner launch file
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(joint_planner_launch)
        # ),
        # # Optionally, add more nodes or actions here
    ])