cmake_minimum_required(VERSION 2.8.3)
project(simon_says_msgs)

add_compile_options(-std=c++11)

##################
## Dependencies ##
##################

find_package(catkin REQUIRED COMPONENTS
    message_generation
)

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

catkin_package(CATKIN_DEPENDS message_runtime)
