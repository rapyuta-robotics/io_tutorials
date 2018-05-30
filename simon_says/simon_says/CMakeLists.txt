cmake_minimum_required(VERSION 2.8.3)
project(simon_says)

add_compile_options(-std=c++11)

##################
## Dependencies ##
##################

set(PKG_DEPS
    roscpp
    std_msgs
    geometry_msgs
)

find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS} message_generation)

#####################
## Code generation ##
#####################

add_message_files(FILES
    DeviceId.msg
    Mode.msg
    SetMode.msg
    Status.msg
    StatusList.msg
)

generate_messages()

############
## catkin ##
############

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${PKG_DEPS} message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(central_node src/device.cpp src/central_node.cpp)
target_link_libraries(central_node ${catkin_LIBRARIES})
add_dependencies(central_node ${PROJECT_NAME}_generate_messages_cpp)

add_executable(gopigo_node src/gopigo_node.cpp)
target_link_libraries(gopigo_node ${catkin_LIBRARIES})
add_dependencies(gopigo_node ${PROJECT_NAME}_generate_messages_cpp)

add_executable(turtle_bot_node src/turtle_bot_node.cpp)
target_link_libraries(turtle_bot_node ${catkin_LIBRARIES})
add_dependencies(turtle_bot_node ${PROJECT_NAME}_generate_messages_cpp)

#############
## Install ##
#############

install(TARGETS central_node gopigo_node turtle_bot_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
