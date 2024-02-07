#include <iostream>
#include <Eigen/Eigen>
// #include "ceres/ceres.h"
#include <ros/ros.h>
#include <vector>
#include "swarm_posefusion/swarm_types.hpp"

class SwarmPosefusionSolver
{
public:
  SwarmPosefusionSolver() {}
  ~SwarmPosefusionSolver() {}
  void init(ros::NodeHandle &nh);
  // std::vector<keyframe> slide_win
private:
  enum SOLVE_TYPE
  {
    EKF = 1,
    GRAPH = 2
  };
  int solve_type_;

  int uav_num_;
  int ugv_num_;

  int self_id_;
  bool self_init_;
  std::string frame_id_;

  std::vector<DataManager> uav_crepes_data_;
  std::vector<DataManager> ugv_crepes_data_;

  ros::Subscriber uav_odom_sub_;
  ros::Subscriber ugv_odom_sub_;
  ros::Subscriber wheel_odom_sub_;

  ros::Publisher added_odom_pub_;
  ros::Publisher fused_odom_pub_;
  ros::Publisher predict_odom_pub_;

  // about filter
  PoseEKF::Ptr odom_filter_;

  // filter record
  double last_wheel_time_;
  double last_uav_time_;
  double last_ugv_time_;
  Eigen::Matrix<double, 6, 1> twist_input_;
  // filter flag
  bool filter_init_;
  bool relative_flag_ = true;
  int relative_cout_ = 0;

  void ugvOdomCallback(const nav_msgs::OdometryConstPtr &msg);
  void uavOdomCallback(const nav_msgs::OdometryConstPtr &msg);
  void wheelOdomCallback(const nav_msgs::OdometryConstPtr &msg);

  void pubFilterOdom(ros::Time stamp);
};
