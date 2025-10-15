
add_library(fsm STATIC FSM.h FSM.cpp)

target_include_directories(fsm PUBLIC event_manager ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(fsm event_manager)