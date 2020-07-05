cmake_minimum_required(VERSION 3.0.2)
project(path_planning_tutorial)

## Compile options
# SET(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_BUILD_TYPE Release)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    ADD_COMPILE_OPTIONS(-std=c++11)
else()
    ADD_COMPILE_OPTIONS(-std=c++11 -O3)
endif()

## Find catkin macros and libraries
FIND_PACKAGE(catkin REQUIRED COMPONENTS
    roscpp
    nav_msgs
    geometry_msgs
    visualization_msgs
    tf
)

## System dependencies are found with CMake's conventions
FIND_PACKAGE(superray_quadmap REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES path_planning_tutorial
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${QUADMAP_INCLUDE_DIRS}
)

SET(COLLISION_DETECTOR_SOURCES
    src/collision_detector/collision_detector.cpp
    include/collision_detector/collision_detector.h
    include/collision_detector/primitive_box.h
    include/collision_detector/primitive_circle.h
)

SET(ASTAR_PLANNER_2D_CONFIGURATION_SOURCES
    src/astar_planner/in_2d_configuration_space/astar_planner_2d.cpp
    include/astar_planner/in_2d_configuration_space/astar_planner_2d.h
    include/astar_planner/in_2d_configuration_space/astar_planner_2d_node.h
    include/astar_planner/in_2d_configuration_space/configuration_2d.h
    ${COLLISION_DETECTOR_SOURCES}
)

ADD_EXECUTABLE(test_collision_detector
    src/test_collision_detector_node.cpp

    include/path_planning_tutorial/utils.h
    include/path_planning_tutorial/visualization.h

    ${COLLISION_DETECTOR_SOURCES}
)
TARGET_LINK_LIBRARIES(test_collision_detector
    ${catkin_LIBRARIES}
    ${QUADMAP_LIBRARIES}
)

ADD_EXECUTABLE(astar_planner_tutorial_1
    src/astar_planner_tutorial_1.cpp

    include/path_planning_tutorial/utils.h
    include/path_planning_tutorial/visualization.h

    ${ASTAR_PLANNER_2D_CONFIGURATION_SOURCES}
)
TARGET_LINK_LIBRARIES(astar_planner_tutorial_1
    ${catkin_LIBRARIES}
    ${QUADMAP_LIBRARIES}
)