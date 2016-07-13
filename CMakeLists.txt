cmake_minimum_required(VERSION 2.8.3)
project(straf_recovery)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(PCL REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  angles
  base_local_planner
  dynamic_reconfigure
  geometry_msgs
  nav_core
  std_msgs
  roscpp
)

generate_dynamic_reconfigure_options(cfg/StrafRecovery.cfg)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    angles
    base_local_planner
    dynamic_reconfigure
    geometry_msgs
    nav_core
    std_msgs
    roscpp
  DEPENDS pcl
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

add_library(obstacle_finder
  src/obstacle_finder.cpp
)

add_executable(obstacle_visualizer
  src/obstacle_visualizer.cpp
)

add_library(straf_recovery
  src/straf_recovery.cpp
)

target_link_libraries(straf_recovery
  obstacle_finder
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(obstacle_visualizer
  obstacle_finder
  ${catkin_LIBRARIES}
)

add_dependencies(straf_recovery ${PROJECT_NAME}_gencfg)
