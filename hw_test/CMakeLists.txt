cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 NEW)
project(hw_test)
add_compile_options(-std=c++17)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mavros_msgs
  nav_msgs
)


catkin_package(
  #INCLUDE_DIRS include# 헤더 파일 포함
  CATKIN_DEPENDS roscpp rospy std_msgs mavros_msgs nav_msgs
#  DEPENDS system_lib
)



include_directories(
  include  
  ${catkin_INCLUDE_DIRS}
)

# Add executable targets
add_executable(vel_x src/vel_x.cpp)
add_executable(vel_x_offb src/vel_x_offb.cpp)
add_executable(vel_x_diff src/vel_x_diff.cpp)




# Link libraries to executable targets
target_link_libraries(vel_x ${catkin_LIBRARIES})
target_link_libraries(vel_x_offb ${catkin_LIBRARIES})
target_link_libraries(vel_x_diff ${catkin_LIBRARIES})


# Add dependencies
add_dependencies(vel_x hw_test_generate_messages_cpp)
add_dependencies(vel_x_offb hw_test_generate_messages_cpp)
add_dependencies(vel_x_diff hw_test_generate_messages_cpp)


