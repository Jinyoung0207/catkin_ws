cmake_minimum_required(VERSION 3.0.2)
project(cps_ugv_formation_control)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  visualization_msgs
  sensor_msgs
)
## Find CasADi package
find_package(casadi REQUIRED)

## Declare a catkin package
catkin_package()

## Build test_node
include_directories(include 
	${catkin_INCLUDE_DIRS}
	${CASADI_INCLUDE_DIRS}
)

add_executable(mpc_ros1_rbt1_node src/mpc_ros1_rbt1.cpp)
add_dependencies(mpc_ros1_rbt1_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_ros1_rbt1_node ${catkin_LIBRARIES} casadi)

add_executable(mpc_ros1_rbt2_node src/mpc_ros1_rbt2.cpp)
add_dependencies(mpc_ros1_rbt2_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_ros1_rbt2_node ${catkin_LIBRARIES} casadi)

add_executable(mpc_ros1_rbt3_node src/mpc_ros1_rbt3.cpp)
add_dependencies(mpc_ros1_rbt3_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_ros1_rbt3_node ${catkin_LIBRARIES} casadi)

add_executable(mpc_ros1_rbt4_node src/mpc_ros1_rbt4.cpp)
add_dependencies(mpc_ros1_rbt4_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_ros1_rbt4_node ${catkin_LIBRARIES} casadi)

add_executable(mpc_ros1_rbt5_node src/mpc_ros1_rbt5.cpp)
add_dependencies(mpc_ros1_rbt5_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_ros1_rbt5_node ${catkin_LIBRARIES} casadi)

#add_executable(mpc_ros1_jackal_node src/mpc_ros1_jackal.cpp)
#add_dependencies(mpc_ros1_jackal_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(mpc_ros1_jackal_node ${catkin_LIBRARIES} casadi)

add_executable(ref_path_make_node src/ref_path_make.cpp)
add_dependencies(ref_path_make_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ref_path_make_node ${catkin_LIBRARIES})

add_executable(ref_path_make2_node src/ref_path_make2.cpp)
add_dependencies(ref_path_make2_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ref_path_make2_node ${catkin_LIBRARIES})

add_executable(ref_path_make3_node src/ref_path_make3.cpp)
add_dependencies(ref_path_make3_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ref_path_make3_node ${catkin_LIBRARIES})

add_executable(ref_path_make4_node src/ref_path_make4.cpp)
add_dependencies(ref_path_make4_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ref_path_make4_node ${catkin_LIBRARIES})

add_executable(ref_path_make5_node src/ref_path_make5.cpp)
add_dependencies(ref_path_make5_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ref_path_make5_node ${catkin_LIBRARIES})

add_executable(global_path_make_node src/global_path_make.cpp)
add_dependencies(global_path_make_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(global_path_make_node ${catkin_LIBRARIES})

add_executable(global_path_make2_node src/global_path_make2.cpp)
add_dependencies(global_path_make2_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(global_path_make2_node ${catkin_LIBRARIES})

add_executable(global_path_make3_node src/global_path_make3.cpp)
add_dependencies(global_path_make3_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(global_path_make3_node ${catkin_LIBRARIES})

add_executable(global_path_make4_node src/global_path_make4.cpp)
add_dependencies(global_path_make4_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(global_path_make4_node ${catkin_LIBRARIES})

add_executable(global_path_make5_node src/global_path_make5.cpp)
add_dependencies(global_path_make5_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(global_path_make5_node ${catkin_LIBRARIES})

#add_executable(read_mavros_topic_node src/read_mavros_topic.cpp)
#add_dependencies(read_mavros_topic_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(read_mavros_topic_node ${catkin_LIBRARIES})

install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME})
