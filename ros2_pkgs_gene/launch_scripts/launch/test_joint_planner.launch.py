from distutils.command import config
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
    launch_remote_controller = IncludeLaunchDescription(PythonLaunchDescriptionSource(remote_controller_launch))


    # planner
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
    topics = ['/joint_cmds', '/joint_states', '/vectornav/raw/common', '/joy']
    rosbag2 = ExecuteProcess(
        cmd=['ros2', 'bag', 'record'] + topics + ['--max-bag-size', '1073741824'],
        output='screen'
    )


    # ros bridge
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="9090",
        description="Port for rosbridge WebSocket server"
    )
    address_arg = DeclareLaunchArgument(
        "address",
        default_value="0.0.0.0",
        description="Bind address (0.0.0.0 to allow external connections)"
    )

    # Node
    rosbridge_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        parameters=[{
            "port": LaunchConfiguration("port"),
            # "address": LaunchConfiguration("address"),
        }]
    )

    rosapi_node = Node(
        package="rosapi",
        executable="rosapi_node",
        name="rosapi",
        output="screen"
    )

    return LaunchDescription([
        # Include the motor driver launch file
        # ExecuteProcess(
        #     cmd=[
        #         '/usr/bin/env', '-i', 
        #         'bash', '-lc',
        #         'source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && roslaunch EtherCAT_test latest_motor_driver.launch',
        #     ],
        #     output='screen'
        # ),

        # Include the IMU
        # launch_vector_nav,

        # Include the joint planner
        delayed_bridge,

        # Logging and remote controller
        # rosbag2,
        launch_remote_controller,

        # # Foxglove bridge
        # port_arg,
        # address_arg,
        # rosbridge_node,
        # rosapi_node
    ])