cmake_minimum_required(VERSION 2.8.3)
project(map_listener)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
)

catkin_package(
  LIBRARIES map_listener
  CATKIN_DEPENDS rospy nav_msgs
)

###########
## Build ##
###########

#############
## Install ##
#############
install(PROGRAMS
    scripts/listener.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

