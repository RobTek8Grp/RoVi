# CMakeLists.txt
cmake_minimum_required(VERSION 2.8.3)
project(rx60_wrapper)

# Use C++ 11
ADD_DEFINITIONS(-std=c++0x)


# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp actionlib actionlib_msgs genmsg)
find_package(Boost REQUIRED COMPONENTS system)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

catkin_package()

# Add Executable
add_executable(${PROJECT_NAME} src/main.cpp src/RX60wrapper.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} rx60controller_gencpp)
