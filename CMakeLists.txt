cmake_policy(VERSION 2.8.12)
cmake_minimum_required(VERSION 2.8.12)
project(rostune)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)


find_package(catkin REQUIRED COMPONENTS std_msgs topic_tools)

add_message_files(DIRECTORY msg FILES SingleNodeStats.msg MultipleNodeStats.msg SingleTopicStats.msg MultipleTopicStats.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(rostune src/main.cpp src/nodestats.cpp src/topicstats.cpp)
target_link_libraries(rostune ${catkin_LIBRARIES})


