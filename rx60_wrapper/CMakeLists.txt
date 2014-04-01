# CMakeLists.txt
cmake_minimum_required(VERSION 2.8.3)
project(rx60_wrapper)

# Use C++ 11
ADD_DEFINITIONS(-std=c++0x)


# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs actionlib actionlib_msgs genmsg message_generation)
find_package(Boost REQUIRED COMPONENTS system)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
add_service_files(FILES command.srv)
generate_messages(DEPENDENCIES std_msgs)
catkin_package()

# Add Executable
add_executable(${PROJECT_NAME} src/main.cpp src/RX60wrapper.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
