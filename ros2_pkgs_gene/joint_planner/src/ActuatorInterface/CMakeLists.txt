
add_library(actuator_interface STATIC ROSActuatorInterface.h ROSActuatorInterface.cpp)

target_include_directories(actuator_interface PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
ament_target_dependencies(actuator_interface rclcpp std_msgs sensor_msgs interfaces)



