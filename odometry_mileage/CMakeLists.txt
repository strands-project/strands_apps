cmake_minimum_required(VERSION 2.8.3)
project(odometry_mileage)

find_package(catkin REQUIRED COMPONENTS
  mongodb_store
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  mongodb_store
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(odometry_mileage src/odometry_mileage.cpp)

add_dependencies(odometry_mileage mongodb_store_generate_messages_cpp)

target_link_libraries(odometry_mileage
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS odometry_mileage
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
