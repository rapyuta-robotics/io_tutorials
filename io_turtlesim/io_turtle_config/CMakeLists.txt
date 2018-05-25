cmake_minimum_required(VERSION 2.8.3)

project(io_turtle_config)

##################
## Dependencies ##
##################

find_package(catkin REQUIRED COMPONENTS)

############
## Catkin ##
############

catkin_package()

#############
## Install ##
#############

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
