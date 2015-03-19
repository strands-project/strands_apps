cmake_minimum_required(VERSION 2.8.3)
project(topic_republisher)

find_package(catkin REQUIRED COMPONENTS 
    geometry_msgs 
    rospy
    sensor_msgs
    )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS geometry_msgs rospy sensor_msgs
)

#############
## Install ##
#############

install(PROGRAMS
  scripts/republish_ptu_jointstate.py
  scripts/republish_robot_pose.py
  scripts/republish_ptu_state.py
  scripts/republish_odometry.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
