cmake_minimum_required(VERSION 3.0.2)
project(EtherCAT_test)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  soem
  std_msgs
  message_generation
)

add_service_files(
  FILES
  StartControl.srv
)

# ---- basics for Eigen start(1) ---- #
find_package(cmake_modules REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

generate_messages(
  DEPENDENCIES
  std_msgs
)

add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES EtherCAT_test
  CATKIN_DEPENDS roscpp std_msgs
  CATKIN_DEPENDS message_runtime roscpp std_msgs
  DEPENDS system_lib Eigen3 # must contain Eigen3 to use Eigen
)

include_directories(
  ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS}
)
# ---- basics for Eigen end(1) ---- #

# ---- basics for Xenomai start(1) ---- #
include_directories(
  ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} include
  /usr/xenomai/include 
  /usr/xenomai/include/cobalt
)

link_directories(
/usr/xenomai/lib
)

link_libraries(
    alchemy
    cobalt
    copperplate
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)

execute_process(COMMAND /usr/xenomai/bin/xeno-config --posix --alchemy --cflags OUTPUT_VARIABLE XENO_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND /usr/xenomai/bin/xeno-config --posix --alchemy --ldflags OUTPUT_VARIABLE XENO_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}${XENO_CFLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${XENO_LDFLAGS}")

# ---- basics for Xenomai end(1) ---- #
# add_executable(latest_motor_driver src/latest_motor_driver.cpp include/Whole_EtherCAT_DC.cpp)
# add_dependencies(latest_motor_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(latest_motor_driver ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

add_executable(latest_motor_driver src/latest_motor_driver_2.cpp include/Whole_EtherCAT_DC_2.cpp)
add_dependencies(latest_motor_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(latest_motor_driver ${catkin_LIBRARIES} ${Eigen3_LIBRARIES})

