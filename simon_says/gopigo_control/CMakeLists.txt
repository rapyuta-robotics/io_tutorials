cmake_minimum_required(VERSION 2.8.3)
project(gopigo_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
)

catkin_package()

install(
  PROGRAMS
    scripts/gopigo_control_node.py
    scripts/gopigo_teleop_key.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
