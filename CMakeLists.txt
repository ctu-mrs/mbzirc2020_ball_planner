cmake_minimum_required(VERSION 2.8.3)
project(balloon_planner)

set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O0 -g -fvar-tracking -fvar-tracking-assignments -DEIGEN_INITIALIZE_MATRICES_BY_NAN")
set(CMAKE_CXX_FLAGS_DEBUG "-std=c++17 -Wall -O0 -g -fvar-tracking -fvar-tracking-assignments -DEIGEN_INITIALIZE_MATRICES_BY_NAN")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++17 -Wall -O0 -g -fvar-tracking -fvar-tracking-assignments -DEIGEN_INITIALIZE_MATRICES_BY_NAN")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-std=c++17 -Wall -O0 -g -fvar-tracking -fvar-tracking-assignments -DEIGEN_INITIALIZE_MATRICES_BY_NAN")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  mrs_msgs
  mrs_lib
  pcl_ros
  cv_bridge
  message_generation
  balloon_filter
  )

find_package(OpenCV 3.2 REQUIRED)

# include Eigen3
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  config/PlanningParams.cfg
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES BalloonPlanner
  CATKIN_DEPENDS
  #  DEPENDS system_lib
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  )

## Declare a C++ executable
add_library(BalloonPlanner
  src/BalloonPlanner.cpp
  )
add_dependencies(BalloonPlanner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
target_link_libraries(BalloonPlanner
  ${catkin_LIBRARIES}
  ${OpenCV_INCLUDE_DIRS}
  )
