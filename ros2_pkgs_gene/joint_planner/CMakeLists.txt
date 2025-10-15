cmake_minimum_required(VERSION 3.8)
project(joint_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH}; /home/gene/DanielWorkspace/ros2_ws/onnxruntime-linux-x64-1.21.0/lib/cmake)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rmw REQUIRED)
find_package(onnxruntime REQUIRED CONFIG)
find_package(Eigen3 REQUIRED)
find_package(vectornav_msgs REQUIRED)


# build subdirectories
add_subdirectory(src/ActuatorInterface)
add_subdirectory(src/DataInterface)
add_subdirectory(src/Planner)
add_subdirectory(src/EventManager)
add_subdirectory(src/FSM)
add_subdirectory(src/StateMachine)
add_subdirectory(src/RLPolicy)

# build executables
add_executable(joint_planner src/joint_planner_node.cpp)
ament_target_dependencies(joint_planner
  rclcpp
  std_msgs
  sensor_msgs
  interfaces
)
target_link_libraries(joint_planner
  actuator_interface
  data_interface
  fsm
  event_manager
  planner_sm
  rl_policy
  planner
)

# install executables
install(TARGETS joint_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY motion_files
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
