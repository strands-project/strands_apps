cmake_minimum_required(VERSION 2.8.3)
project(reconfigure_inflation)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  dynamic_reconfigure
  move_base_msgs
  rospy
)

#catkin_python_setup()

catkin_package()

#include_directories(
  #${catkin_INCLUDE_DIRS}
#)

#############
## Install ##
#############

 install(PROGRAMS
   scripts/reconfigure_inflation_server.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
