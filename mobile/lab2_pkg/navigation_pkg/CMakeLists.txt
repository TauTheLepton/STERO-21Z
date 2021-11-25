cmake_minimum_required(VERSION 3.0.2)
project(navigation_pkg)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  move_base 
)

# catkin_python_setup()

catkin_package(
   INCLUDE_DIRS
   CATKIN_DEPENDS geometry_msgs rospy
)

include_directories(
${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)