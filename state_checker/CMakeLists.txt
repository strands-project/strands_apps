cmake_minimum_required(VERSION 2.8.3)
project(state_checker)

find_package(catkin REQUIRED COMPONENTS 
    geometry_msgs
    nav_msgs
    roscpp 
    rospy 
    std_msgs
    )

catkin_python_setup()
catkin_package()

#############
## Install ##
#############

 install(PROGRAMS
   scripts/state_checker.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
