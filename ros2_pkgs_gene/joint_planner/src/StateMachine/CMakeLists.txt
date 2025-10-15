add_library(planner_sm STATIC StateMachine.h StateMachine_DataCollection.cpp)

target_include_directories(planner_sm PUBLIC fsm actuator_interface data_interface event_manager planner rl_policy ${CMAKE_CURRENT_SOURCE_DIR})
ament_target_dependencies(planner_sm
  rclcpp
)
target_link_libraries(planner_sm fsm actuator_interface data_interface event_manager planner rl_policy)