cmake_minimum_required(VERSION 3.5)
project(ros2_xenomai_bridge)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interfaces REQUIRED)

# add executable
add_executable(motor_state_pub src/motor_state_publisher.cpp)
target_link_libraries(motor_state_pub
  rt
)
ament_target_dependencies(motor_state_pub
  rclcpp
  sensor_msgs
)

add_executable(motor_cmd_sub src/motor_command_subscriber.cpp)
target_link_libraries(motor_cmd_sub
  rt
)
ament_target_dependencies(motor_cmd_sub
  rclcpp
  interfaces
  sensor_msgs
)

# install executables
install(TARGETS
  motor_state_pub
  motor_cmd_sub
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
