cmake_minimum_required(VERSION 2.8.3)
project(static_transform_manager)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  rospy
  std_msgs
  tf
)


## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetTransformation.srv
  StopTransformation.srv
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES static_transform_manager
#  CATKIN_DEPENDS geometry_msgs message_generation rospy std_msgs tf
#  DEPENDS system_lib
)

install(PROGRAMS
  scripts/static_tf_services.py
  scripts/rviz_click_to_tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
