cmake_minimum_required(VERSION 2.8.3)
project(sawyer_demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  intera_core_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  intera_core_msgs
  geometry_msgs
)


#############
## Install ##
#############

catkin_python_setup()

install(PROGRAMS
   scripts/burger.py
   scripts/manipulate.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)