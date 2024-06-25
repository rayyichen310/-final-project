cmake_minimum_required(VERSION 3.0.2)
project(auto_nav)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  amcl
  actionlib
  costmap_2d
  geometry_msgs
  gmapping
  map_server
  move_base
  move_base_msgs
  roscpp
  rospy
  roslaunch
  std_msgs
  std_srvs
  nav_msgs
  tf
  angles
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS amcl actionlib costmap_2d gmapping map_server move_base roscpp std_msgs geometry_msgs move_base_msgs nav_msgs tf angles
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/path_planning.py
  scripts/next_goal.py
  scripts/test_goal.py
  scripts/test_path.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############



