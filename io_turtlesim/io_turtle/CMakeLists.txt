cmake_minimum_required(VERSION 2.8.3)
project(io_turtle)

##################
## Dependencies ##
##################

add_compile_options(-std=c++11)

set(PKG_DEPS
    roscpp
    geometry_msgs
    dynamic_reconfigure
    io_turtle_action
    io_turtle_msgs
    io_turtle_services
)

find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS})

generate_dynamic_reconfigure_options(
    cfg/Control.cfg
)

catkin_package(
    CATKIN_DEPENDS ${PKG_DEPS}
    INCLUDE_DIRS include
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
    src/io_turtle.cpp
    src/io_turtle_node.cpp  
)

add_dependencies(${PROJECT_NAME}_node
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY cfg
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
