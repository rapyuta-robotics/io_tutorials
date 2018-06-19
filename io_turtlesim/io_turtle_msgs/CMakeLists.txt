cmake_minimum_required(VERSION 2.8.3)
project(io_turtle_msgs)

##################
## Dependencies ##
##################

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    message_generation
)

#####################
## Code generation ##
#####################

add_message_files(FILES 
    DistanceSensor.msg
    Pose.msg
    Velocity.msg
)

generate_messages(DEPENDENCIES
    std_msgs
)

############
## Catkin ##
############

catkin_package(CATKIN_DEPENDS
    roscpp
    std_msgs
    message_runtime
)
