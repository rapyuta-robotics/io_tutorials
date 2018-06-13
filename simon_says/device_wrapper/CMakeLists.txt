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
    LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/device_wrapper.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
