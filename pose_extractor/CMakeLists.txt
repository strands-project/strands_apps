cmake_minimum_required(VERSION 2.8.3)
project(pose_extractor)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

#############
## Install ##
#############

 install(PROGRAMS
   scripts/extract_last_pose_from_path.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
