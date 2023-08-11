cmake_minimum_required(VERSION 3.0.2)
project(kbot_odom)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  kbot_msgs
  nav_msgs
  robot_state_publisher
  roscpp
  std_msgs
  tf
)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES kbot_odom
 CATKIN_DEPENDS geometry_msgs kbot_msgs nav_msgs robot_state_publisher roscpp std_msgs tf
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(kbot_odom_node src/kbot_odom.cpp)
add_dependencies(kbot_odom_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kbot_odom_node ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS kbot_odom_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############
