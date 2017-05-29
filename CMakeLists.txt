cmake_minimum_required(VERSION 2.8.3)
project(rostune)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

add_message_files(DIRECTORY msg FILES Num.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(rostune src/main.cpp)
target_link_libraries(rostune ${catkin_LIBRARIES})
add_dependencies(rostune main_cpp)

