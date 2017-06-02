cmake_minimum_required(VERSION 2.8.3)
project(rostune)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)


find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

add_message_files(DIRECTORY msg FILES Logline.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(rostune src/main.cpp src/nodestats.cpp)
target_link_libraries(rostune ${catkin_LIBRARIES})
add_dependencies(rostune msg_pkg_generate_messages_cpp main_cpp nodestats_cpp nodestats_h)

add_executable(rostune_logger src/logger.cpp)
target_link_libraries(rostune_logger ${catkin_LIBRARIES})
add_dependencies(rostune_logger msg_pkg_generate_messages_cpp logger_cpp)

