cmake_minimum_required(VERSION 3.0.2)
project(hakuroukun_communication)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hakuroukun_communication

CATKIN_DEPENDS rospy std_msgs #edge_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/hakuroukun_communication_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Install ##
#############
