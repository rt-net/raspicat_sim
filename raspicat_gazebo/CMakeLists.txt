cmake_minimum_required(VERSION 3.0.2)
project(raspicat_gazebo)

set(CMAKE_CXX_FLAGS "-std=c++14 -O3 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  gazebo_plugins
  gazebo_ros_control
  xacro
  raspimouse_msgs
)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ultrasonic_convert
  src/ultrasonic_sensor_real_data_convert.cpp
)
add_dependencies(ultrasonic_convert
  ${TARGET_NAME} raspimouse_msgs_generate_messages_cpp
)
target_link_libraries(ultrasonic_convert
  ${catkin_LIBRARIES}
)

add_executable(motors_sim
  src/motors_sim.cpp
)
target_link_libraries(motors_sim
  ${catkin_LIBRARIES}
)

install(TARGETS ultrasonic_convert motors_sim
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(PROGRAMS
  scripts/download_gazebo_models.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
