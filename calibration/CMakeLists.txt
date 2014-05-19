cmake_minimum_required(VERSION 2.8.3)
project(calibration)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs pcl_ros group4_msgs)

find_package(PCL REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(calibration src/calibration_node.cpp)
target_link_libraries(calibration ${catkin_LIBRARIES} ${PCL_LIBRARIES})


