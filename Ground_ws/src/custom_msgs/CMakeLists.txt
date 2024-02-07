cmake_minimum_required(VERSION 3.0.2)
project(custom_msgs)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    message_filters
    message_generation
)

add_message_files(
  FILES
  blind_info.msg
  map_info.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES msg_and_srv
#  CATKIN_DEPENDS depends
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
#  ${catkin_INCLUDE_DIRS}
)