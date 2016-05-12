cmake_minimum_required(VERSION 2.8.3)
project(door_pass)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    actionlib
    actionlib_msgs 
    geometry_msgs
    message_generation 
    move_base_msgs
    nav_msgs 
    roscpp 
    rospy 
    sensor_msgs
    std_msgs 
    tf
    mongodb_store
    )
set ( CMAKE_CXX_FLAGS "-ggdb")

catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  FILES
  DoorCheckStat.msg
  DoorWaitStat.msg
)

add_service_files(
  FILES
  PredictDoorState.srv
)

add_action_files(
    DIRECTORY action 
    FILES 
    DoorCheck.action
    DoorWait.action
    BuildPrediction.action
)

generate_messages(
    DEPENDENCIES
    std_msgs 
    actionlib_msgs
    geometry_msgs
)

catkin_package(
   INCLUDE_DIRS include
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(CDoorDetection src/door_pass/CDoorDetection.cpp)

add_executable(door_pass src/door_pass/door_pass.cpp)
add_executable(door_detect src/door_pass/door_detect.cpp)
target_link_libraries (door_pass  CDoorDetection)
target_link_libraries (door_detect  CDoorDetection)

add_dependencies(CDoorDetection ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(door_pass ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(door_detect ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries (door_pass  ${catkin_LIBRARIES})
target_link_libraries (door_detect  ${catkin_LIBRARIES})

#############
## Install ##
#############

 install(PROGRAMS
   scripts/door_passing.py
   scripts/door_checking.py
   scripts/door_wait_and_pass.py
   scripts/door_waiting.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

 install(TARGETS CDoorDetection door_pass door_detect
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
