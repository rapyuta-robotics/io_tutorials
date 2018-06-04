cmake_minimum_required(VERSION 2.8.3)
project(simon_says_device_wrapper)

add_compile_options(-std=c++11)

##################
## Dependencies ##
##################

set(PKG_DEPS
    roscpp
    simon_says_msgs
)

find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS})

############
## catkin ##
############

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS ${PKG_DEPS}
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} include/)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp"
)
