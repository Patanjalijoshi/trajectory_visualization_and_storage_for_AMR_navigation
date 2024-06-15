cmake_minimum_required(VERSION 3.0.2)
project(my_cpp_package)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  roslib
)

## Add service files
add_service_files(
  FILES
  SaveTrajectory.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Define catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

## Include directories for header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable for trajectory_publisher_saver_node
add_executable(trajectory_publisher_saver_node src/trajectory_publisher_saver.cpp)

## Declare a C++ executable for trajectory_reader_publisher_node
add_executable(trajectory_reader_publisher_node src/trajectory_reader_publisher.cpp)

## Add dependencies to the executables
add_dependencies(trajectory_publisher_saver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(trajectory_reader_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Link libraries to the executables
target_link_libraries(trajectory_publisher_saver_node ${catkin_LIBRARIES})
target_link_libraries(trajectory_reader_publisher_node ${catkin_LIBRARIES})
