cmake_minimum_required(VERSION 2.8.3)
project(swarm_bridge)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)

set(ENABLE_TCP true) # requires zmq, zmqpp 

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  traj_utils
  custom_msgs
)

catkin_package(
  CATKIN_DEPENDS traj_utils custom_msgs
)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(bridge_udp_node
  src/bridge_udp_node.cpp 
  )
target_link_libraries(bridge_udp_node
  ${catkin_LIBRARIES}
  )
add_dependencies(bridge_udp_node ${catkin_EXPORTED_TARGETS})

if(ENABLE_TCP)

  add_executable(bridge_tcp_node
    src/bridge_tcp_node.cpp 
    )
  target_link_libraries(bridge_tcp_node
    ${catkin_LIBRARIES}
    zmq
    zmqpp
    )

endif(ENABLE_TCP)


