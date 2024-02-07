#ifndef _UGV_SERVE_PLAN_H_
#define _UGV_SERVE_PLAN_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <nav_msgs/Odometry.h>
struct BlindUGVInfo
{
  /* info of Blind UGV in need*/
  int robo_id;
  ros::Time colli_time;
  Eigen::Vector3d colli_pos;
};

class ServePlan
{

  public:
    void init(ros::NodeHandle &nh);

  private:
    ros::Subscriber ugv_info_sub_;
    ros::Publisher waypoint_pub_;
    std::vector<BlindUGVInfo> ugv_info_buf_;

    void UGVinfoCallback(const nav_msgs::OdometryConstPtr &msg);
};

#endif