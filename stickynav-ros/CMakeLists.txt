cmake_minimum_required(VERSION 3.0.0)
project(stickynav-ros)

find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14)

catkin_package()

cs_add_executable(online_planner_ros_node
        src/online_planner_ros.cpp
        )
target_link_libraries(online_planner_ros_node ${catkin_LIBRARIES})