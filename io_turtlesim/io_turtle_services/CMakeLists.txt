cmake_minimum_required(VERSION 2.8.3)
project(io_turtle_services)

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

add_service_files(FILES
    RegisterSimTurtle.srv
    RegisterTurtle.srv
    TeleportTurtle.srv
)

generate_messages()

############
## Catkin ##
############

catkin_package(CATKIN_DEPENDS message_runtime)
