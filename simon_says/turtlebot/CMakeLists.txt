cmake_minimum_required(VERSION 2.8.3)
project(turtlebot)

add_compile_options(-std=c++11)

##################
## Dependencies ##
##################

set(PKG_DEPS
    roscpp
    simon_says_msgs
    simon_says_device_wrapper
)

find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS})

############
## catkin ##
############

catkin_package(CATKIN_DEPENDS ${PKG_DEPS})

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/turtle_bot_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
