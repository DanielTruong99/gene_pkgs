
add_library(event_manager STATIC EventManager.h EventManager.cpp)

target_include_directories(event_manager PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
