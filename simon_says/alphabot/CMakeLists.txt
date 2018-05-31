cmake_minimum_required(VERSION 2.8.3)
project(alphabot)

##################
## Dependencies ##
##################

set(PKG_DEPS
    rospy
    simon_says_msgs
)

find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS})

############
## catkin ##
############

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
