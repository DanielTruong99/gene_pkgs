add_library(planner STATIC Planner.h Planner.cpp)

target_include_directories(planner PUBLIC data_interface ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(planner
  data_interface
)
