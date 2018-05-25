cmake_minimum_required(VERSION 2.8.3)
project(io_turtle_msgs)

##################
## Dependencies ##
##################

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
    message_generation
)

#####################
## Code generation ##
#####################

add_message_files(FILES 
    Pose.msg
    Velocity.msg
    DistanceSensor.msg
)

generate_messages()

############
## Catkin ##
############

catkin_package(CATKIN_DEPENDS message_runtime)
