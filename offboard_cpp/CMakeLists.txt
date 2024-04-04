cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 NEW)
project(offboard_cpp)
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
  LIBRARIES casadi
  CATKIN_DEPENDS roscpp rospy std_msgs mavros_msgs nav_msgs
#  DEPENDS system_lib
)
# include 내 헤더 파일 경로임 포함시켜줘야됨
find_package(casadi REQUIRED)

include_directories(
  include  
  ${catkin_INCLUDE_DIRS}
  ${casadi_INCLUDE_DIRS}
)
# main함수 파일도 포함시켜줘야됨.

add_executable(send_traj src/send_traj.cpp)
add_executable(velocity_traj src/velocity_traj.cpp)
add_executable(error src/error.cpp)
add_executable(velocity_pub0 src/velocity_pub0.cpp)
add_executable(velocity_pub1 src/velocity_pub1.cpp)
add_executable(velocity_pub2 src/velocity_pub2.cpp)
add_executable(velocity_pub3 src/velocity_pub3.cpp)

add_executable(formation_node src/formation_node.cpp)
add_executable(casadi_test src/nlp_codegen.cpp)




target_link_libraries(send_traj ${catkin_LIBRARIES})
target_link_libraries(velocity_traj ${catkin_LIBRARIES})
target_link_libraries(error ${catkin_LIBRARIES})


target_link_libraries(velocity_pub0 ${catkin_LIBRARIES})
target_link_libraries(velocity_pub1 ${catkin_LIBRARIES})
target_link_libraries(velocity_pub2 ${catkin_LIBRARIES})
target_link_libraries(velocity_pub3 ${catkin_LIBRARIES})
target_link_libraries(formation_node ${catkin_LIBRARIES})

target_link_libraries(casadi_test
  ${catkin_LIBRARIES}
  casadi
  )






add_dependencies(send_traj offboard_cpp_generate_messages_cpp)
add_dependencies(velocity_traj offboard_cpp_generate_messages_cpp)
add_dependencies(error offboard_cpp_generate_messages_cpp)



add_dependencies(velocity_pub0 offboard_cpp_generate_messages_cpp)
add_dependencies(velocity_pub1 offboard_cpp_generate_messages_cpp)
add_dependencies(velocity_pub2 offboard_cpp_generate_messages_cpp)
add_dependencies(velocity_pub3 offboard_cpp_generate_messages_cpp)
add_dependencies(formation_node offboard_cpp_generate_messages_cpp)






