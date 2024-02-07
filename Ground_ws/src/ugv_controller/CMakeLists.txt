cmake_minimum_required(VERSION 2.8.3)
project(mpc)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  std_msgs
  visualization_msgs
  bspline_opt
  traj_utils
)
find_package(Eigen3 REQUIRED)
# find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

add_message_files(FILES
  Polynome.msg
)

generate_messages(DEPENDENCIES
	std_msgs
  geometry_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs bspline_opt traj_utils
 message_runtime
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PROJECT_SOURCE_DIR}/include
)

add_library(mpc
  src/mpc.cpp
  src/cubic_spline_planner.cpp
)

add_dependencies(mpc mpc_gencpp)

target_link_libraries(mpc
    OsqpEigen::OsqpEigen
    ${catkin_LIBRARIES}
    )

add_executable(mpc_controller_node
    src/mpc_node.cpp
  )
target_link_libraries(mpc_controller_node
  mpc
  ${catkin_LIBRARIES}
)
