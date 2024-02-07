#!/bin/bash

if ! pgrep -x "roscore" > /dev/null
then
    roscore &
fi

WORKSPACE_DIR=$(pwd)

MARSIM=$WORKSPACE_DIR/MARSIM_ws
UGV=$WORKSPACE_DIR/Ground_ws
UAV=$WORKSPACE_DIR/Air_ws

cd $MARSIM && catkin_make
cd $UGV && catkin_make
cd $UAV && catkin_make

source $MARSIM/devel/setup.sh && roslaunch test_interface single_drone_vlp32.launch &
source $UGV/devel/setup.sh && roslaunch ego_planner swarm_sim.launch &
source $UAV/devel/setup.sh && roslaunch ego_planner swarm_sim.launch &
source $UAV/devel/setup.sh && roslaunch ego_planner rviz.launch
