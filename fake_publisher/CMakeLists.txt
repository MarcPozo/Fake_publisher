cmake_minimum_required(VERSION 2.8.3)

project(fake_publisher)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  dynamic_reconfigure
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  tf
)

generate_dynamic_reconfigure_options(
   cfg/Error.cfg
 )

## Generate messages in the 'msg' folder
 add_message_files(
  FILES
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
 )

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS dynamic_reconfigure message_runtime rospy std_msgs sensor_msgs geometry_msgs
  DEPENDS
)

add_executable(fake_publisher scripts/fake_publisher.py)
add_dependencies(fake_publisher ${PROJECT_NAME}_gencfg)

set_target_properties(fake_publisher PROPERTIES LINKER_LANGUAGE CMAKE_py_LINK_EXECUTABLE)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)
