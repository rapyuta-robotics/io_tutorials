cmake_minimum_required(VERSION 2.8.3)
project(custom_msgs)

##################
## Dependencies ##
##################

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    message_generation
)

#####################
## Code generation ##
#####################

add_message_files(FILES
   OrderMetric.msg
   LocalizationMetric.msg
   RobotSpeedMetric.msg
   RobotTrajectoryMetric.msg
   RoutesMetric.msg
   UnloadMetric.msg
   ChargeMetric.msg
   ChargeStatus.msg
)

generate_messages(DEPENDENCIES
    std_msgs
)

install(PROGRAMS
  custom_talker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  custom_message.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

############
## Catkin ##
############

catkin_package(CATKIN_DEPENDS
    std_msgs
    message_runtime
)

