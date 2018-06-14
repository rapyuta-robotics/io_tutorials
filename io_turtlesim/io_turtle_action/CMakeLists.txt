cmake_minimum_required(VERSION 2.8.3)
project(io_turtle_action)

##################
## Dependencies ##
##################

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    actionlib
    actionlib_msgs
    message_generation
)

#####################
## Code generation ##
#####################

add_action_files(
    DIRECTORY action
    FILES GoTo.action
)

generate_messages(DEPENDENCIES
    std_msgs
    actionlib_msgs
)

############
## Catkin ##
############

catkin_package(CATKIN_DEPENDS
    roscpp
    std_msgs
    actionlib
    actionlib_msgs
    message_runtime
)
