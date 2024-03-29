cmake_minimum_required(VERSION 3.0.2)
project(bilateral_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  dynamixel_wrapper
)



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
#  LIBRARIES bilateral_control
#  CATKIN_DEPENDS roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dynamixel_simple_control   src/dynamixel_simple_control.cpp)
target_link_libraries(dynamixel_simple_control
  ${catkin_LIBRARIES}
)

add_executable(symmetric   src/symmetric.cpp)
target_link_libraries(symmetric
  ${catkin_LIBRARIES}
)

add_executable(unity_bilateral   src/unity_bilateral.cpp)
target_link_libraries(unity_bilateral
  ${catkin_LIBRARIES}
)


