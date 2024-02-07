#include <iostream>
#include "ros/ros.h"
#include "swarm_posefusion/swarm_posefusion_solver.h"
#include <nav_msgs/Odometry.h>
#include <string>
#include <vector>
#include <map>

int main(int argc, char **argv)
{

  // ROS_INFO("SWARM VO FUSE ROS\nIniting\n");

  // Use time as seed

  ros::init(argc, argv, "swarm_localization");

  ros::NodeHandle nh("~");

  SwarmPosefusionSolver fusenode;
  fusenode.init(nh);

  // ros::MultiThreadedSpinner spinner(4); // Use 4 threads
  //  spinner.spin();
  ros::spin();

  return 0;
}