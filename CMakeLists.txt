cmake_minimum_required(VERSION 2.8.3)
project(controls)

find_package(catkin REQUIRED COMPONENTS 
  message_generation 
  roscpp 
  rospy 
  std_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robotender_server
  CATKIN_DEPENDS message_runtime rospy std_msgs roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  nodes/controls/PID_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)