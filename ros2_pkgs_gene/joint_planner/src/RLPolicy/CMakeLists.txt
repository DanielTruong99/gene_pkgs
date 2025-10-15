add_library(rl_policy STATIC RLPolicy2.cpp RLPolicy2.h)


target_include_directories(rl_policy PUBLIC /home/gene/DanielWorkspace/ros2_ws/onnxruntime-linux-x64-1.21.0/include ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(rl_policy /home/gene/DanielWorkspace/ros2_ws/onnxruntime-linux-x64-1.21.0/lib/libonnxruntime.so)

ament_target_dependencies(rl_policy rclcpp)
