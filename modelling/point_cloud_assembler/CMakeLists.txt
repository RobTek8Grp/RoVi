cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_assembler)

#################################
#	 Catkin specific	#
#################################
catkin_package()

#########################
#	Packages	#
#########################

#	Catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	sensor_msgs
	group4_msgs
	pcl_ros
	tf
	tf_conversions
)

#	PCL
find_package(PCL REQUIRED)

#########################################
#	Includes/links/definitions	#
#########################################
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(-std=c++0x ${PCL_DEFINITIONS})

#################
#     Build 	#
#################

add_executable		(point_cloud_assembler_node src/PointCloudAssembler.cpp src/PointCloudStitching.cpp)
add_dependencies	(point_cloud_assembler_node point_cloud_assembler_generate_messages_cpp)
target_link_libraries	(point_cloud_assembler_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})