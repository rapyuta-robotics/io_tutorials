cmake_minimum_required(VERSION 2.8.3)
project(io_turtle_command_center)

##################
## Dependencies ##
##################

add_compile_options(-std=c++11)

set(PKG_DEPS
    roscpp
    io_turtle_services
)

find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS})

catkin_package(CATKIN_DEPENDS ${PKG_DEPS})

###########
## Build ##
###########

include_directories(
    ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node
    src/io_turtle_command_center_node.cpp
)

add_dependencies(${PROJECT_NAME}_node
    ${catkin_EXPORTED_TARGETS}
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

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
