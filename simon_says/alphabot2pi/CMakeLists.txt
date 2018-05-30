cmake_minimum_required(VERSION 2.8.3)
project(alphabot2pi)

##################
## Dependencies ##
##################

set(PKG_DEPS
  rospy
  std_msgs
  simon_says
)

find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS})

catkin_python_setup()

catkin_package(CATKIN_DEPENDS ${PKG_DEPS})

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

#############
## Install ##
#############

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
