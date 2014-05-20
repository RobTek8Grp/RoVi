cmake_minimum_required(VERSION 2.8.3)
project(surface_reconstruction)

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
	pcl_ros
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

add_executable		(surface_reconstruction_node src/SurfaceReconstruction.cpp)
add_dependencies	(surface_reconstruction_node surface_reconstruction_generate_messages_cpp)
target_link_libraries	(surface_reconstruction_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})