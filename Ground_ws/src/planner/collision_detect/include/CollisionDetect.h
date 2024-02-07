#ifndef _COLLISION_DETECT_H_
#define _COLLISION_DETECT_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <plan_env/grid_map.h>
#include <bspline_opt/uniform_bspline.h>
#include <swarm_posefusion/swarm_types.hpp>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <custom_msgs/blind_info.h>

class CollisionDetect
{
public:
  void init(ros::NodeHandle &nh);
  double caculate_collision(GridMap::Ptr grid_map,
                            ego_planner::UniformBspline bspline,
                            ros::Time bspline_start_time,
                            Eigen::Vector3d odom_pos,
                            Eigen::Vector3d odom_vel,
                            Eigen::Quaterniond odom_orient,
                            Eigen::Matrix<double, 6, 6> odom_cov);
  void visualize_collision();

private:
  std::pair<bool, Eigen::Vector3d> ellipse_check(GridMap::Ptr grid_map,
                                                 Eigen::Vector3d odom_pos,
                                                 Eigen::Matrix<double, 2, 2> odom_cov);


  std::pair<bool, Eigen::Vector3d> ellipse_check(GridMap::Ptr grid_map,
                                                 Eigen::Vector3d odom_pos,
                                                 Eigen::Vector2d ellipse_axis,
                                                 Eigen::Matrix2d eigen_vectors);

  visualization_msgs::Marker ellipse_marker(Eigen::Vector3d odom_pos,
                                            Eigen::Matrix<double, 2, 2> odom_cov,
                                            int id);
                                            
  visualization_msgs::Marker ellipse_marker(Eigen::Vector3d odom_pos,
                                            Eigen::Vector2d ellipse_axis,
                                            Eigen::Matrix2d eigen_vectors,
                                            int id);

  Eigen::Vector3d choosePoint(Eigen::Vector3d pos, Eigen::Vector3d vel);
  Eigen::Vector3d closetPointInBound(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt);
  int isInBound(const Eigen::Vector3d &pos);

  int self_id;
  double det_range_;

  PoseEKF::Ptr ekf;
  ros::Publisher ellipse_pub;
  ros::Publisher collision_point_pub;
  ros::Publisher blind_info_pub;
  visualization_msgs::MarkerArray marker_array;
  std::pair<double, Eigen::Vector3d> last_collision_info;
  Eigen::Vector3d bound_max_, bound_min_;


};

#endif