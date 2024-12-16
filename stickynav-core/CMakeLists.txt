cmake_minimum_required(VERSION 3.0.0)
project(stickynav-core)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++14)

catkin_package()

#############
# LIBRARIES #
#############

#cs_add_library(${PROJECT_NAME}
#  src/data/trajectory_segment.cpp
#  src/online_planner.cpp
#)

#target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

#cs_install()
#cs_export()