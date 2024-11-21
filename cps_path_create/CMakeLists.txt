cmake_minimum_required(VERSION 3.0.2)
project(cps_path_create)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  mavros_msgs
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
# 패키지 선언
catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  rospy 
  std_msgs 
  nav_msgs 
  mavros_msgs
)


function(custom_executable target sources)
  add_executable(${target} ${sources})
  target_link_libraries(${target} ${catkin_LIBRARIES} )
  
  target_include_directories(${target} PRIVATE include)
  
  install(TARGETS ${target}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endfunction()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


custom_executable(global_path_cps_node src/global_path_cps.cpp)

custom_executable(ref_path_creater src/ref_path_creater.cpp)

## Specify additional locations of header files
## Your package locations should be listed before other locations

#launch rviz 파일 설치
install(DIRECTORY launch rviz config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

