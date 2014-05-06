cmake_minimum_required(VERSION 2.8.3)
project(simple_pose_mission)


# find_package(catkin REQUIRED COMPONENTS roscpp moveit_core moveit_ros_planning moveit_ros_planning_interface pluginlib)
find_package(catkin REQUIRED COMPONENTS roscpp moveit_core moveit_ros_planning moveit_ros_planning_interface visualization_msgs interactive_markers group4_msgs)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS}) # does this need to be here?

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simple_pose_mission
#  CATKIN_DEPENDS
#    moveit_core
#    interactive_markers
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
## Declare a cpp executable
add_executable(simple_pose_mission_node src/simple_pose_mission_node.cpp src/marker_pose.cpp)

target_link_libraries(simple_pose_mission_node
   ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES}
)
