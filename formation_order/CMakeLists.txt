cmake_minimum_required(VERSION 3.0.2)
project(formation_order)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PkgConfig REQUIRED)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES arduino_pkg
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(formation_order_node src/formation_order.cpp src/main.cpp src/Hungarian.cpp)
target_link_libraries(formation_order_node
  ${catkin_LIBRARIES}
)
