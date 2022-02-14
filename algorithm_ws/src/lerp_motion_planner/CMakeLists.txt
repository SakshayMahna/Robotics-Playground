cmake_minimum_required(VERSION 3.0.2)
project(lerp_motion_planner)

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
    geometric_shapes
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    pp_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

include_directories(include)
include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(lerp_example  src/lerp_example.cpp)
target_link_libraries(lerp_example
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Lerp planning plugin
add_library(moveit_lerp_planner_plugin
  src/lerp_planner_manager.cpp
  src/lerp_interface.cpp
  src/lerp_planning_context.cpp)
set_target_properties(moveit_lerp_planner_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(moveit_lerp_planner_plugin ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS
    moveit_lerp_planner_plugin
  ARCHIVE DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY
    include
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Mark roslaunch files for installation
install(
  DIRECTORY
    launch
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
#catkin_lint ignore uninstalled_plugin
install(FILES lerp_interface_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc/creating_moveit_plugins/lerp_motion_planner/)
