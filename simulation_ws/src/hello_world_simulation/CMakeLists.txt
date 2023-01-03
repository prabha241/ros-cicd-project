cmake_minimum_required(VERSION 2.8.3)
project(hello_world_simulation)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros  
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  rosgraph_msgs
  turtlebot3_description # required to install .rviz model
)


################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS
    gazebo_ros
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    message_runtime
    rosgraph_msgs
    robomaker_simulation_msgs
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

################################################################################
# Install
################################################################################

install(DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Copy the rviz model for easier access in AWS RoboMaker RViz 
install(FILES ${turtlebot3_description_DIR}/../rviz/model.rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
  RENAME turtlebot3_model.rviz
)
