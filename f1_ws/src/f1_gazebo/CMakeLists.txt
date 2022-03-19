cmake_minimum_required(VERSION 3.0.2)
project(f1_gazebo)

find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    DEPENDS
    CATKIN_DEPENDS
    INCLUDE_DIRS
    LIBRARIES
)


#############
## Install ##
#############

install(DIRECTORY
  launch
  models
  worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
