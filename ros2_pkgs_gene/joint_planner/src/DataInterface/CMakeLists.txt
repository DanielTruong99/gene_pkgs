
add_library(data_interface STATIC ROSDataInterface.h Robot.h ROSDataInterface.cpp)

target_include_directories(data_interface PUBLIC event_manager ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(data_interface Eigen3::Eigen event_manager)
ament_target_dependencies(data_interface rclcpp std_msgs sensor_msgs interfaces vectornav_msgs)



