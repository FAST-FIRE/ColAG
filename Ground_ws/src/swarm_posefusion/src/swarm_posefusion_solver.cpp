#include "swarm_posefusion/swarm_posefusion_solver.h"
#include <tf/tf.h>
using namespace std;
using namespace Eigen;

void SwarmPosefusionSolver::init(ros::NodeHandle &nh)
{
  nh.param("fusion/ugv_num", ugv_num_, 1);
  nh.param("fusion/uav_num", uav_num_, 1);
  nh.param("fusion/self_id", self_id_, 0);
  nh.param("fusion/frame", frame_id_, string("world"));
  nh.param("fusion/solve_type", solve_type_, 1);

  nh.param("fusion/self_init", self_init_, false);

  double q, r;
  nh.param("fusion/q", q, 0.01);
  nh.param("fusion/r", r, 1.0);
  if (solve_type_ = SOLVE_TYPE::EKF)
  {
    odom_filter_.reset(new PoseEKF(q, r));
  }
  uav_crepes_data_.resize(uav_num_);
  ugv_crepes_data_.resize(ugv_num_);

  // ugv_odom_sub_ = nh.subscribe("ugv_odom", 10, &SwarmPosefusionSolver::ugvOdomCallback, this, ros::TransportHints().tcpNoDelay());
  uav_odom_sub_ = nh.subscribe("uav_odom", 10, &SwarmPosefusionSolver::uavOdomCallback, this, ros::TransportHints().tcpNoDelay());
  wheel_odom_sub_ = nh.subscribe("wheel_odom", 10, &SwarmPosefusionSolver ::wheelOdomCallback, this, ros::TransportHints().tcpNoDelay());

  added_odom_pub_ = nh.advertise<nav_msgs::Odometry>("added_odom", 10);
  predict_odom_pub_ = nh.advertise<nav_msgs::Odometry>("predict_odom", 10);
  fused_odom_pub_ = nh.advertise<nav_msgs::Odometry>("fused_odom", 10);

  for (int i = 0; i < uav_num_; i++)
  {
    uav_crepes_data_[i].init(i, self_id_);
    string sub_topic_name = string("crepes") + string("/uav_") + std::to_string(i) + string("_odom");
    uav_crepes_data_[i].relative_odom_sub_ = nh.subscribe(sub_topic_name, 10, &DataManager::relativeOdomCallback, &uav_crepes_data_[i], ros::TransportHints().tcpNoDelay());
  }

  for (int i = 0; i < ugv_num_; i++)
  {
    if (i == self_id_)
      continue;
    ugv_crepes_data_[i].init(i, self_id_);
    string sub_topic_name = string("crepes") + string("/ugv_") + std::to_string(i) + string("_odom");
    ugv_crepes_data_[i].relative_odom_sub_ = nh.subscribe(sub_topic_name, 10, &DataManager::relativeOdomCallback, &ugv_crepes_data_[i], ros::TransportHints().tcpNoDelay());
  }

  filter_init_ = false;

  last_wheel_time_ = 0;
  last_uav_time_ = 0;
  last_ugv_time_ = 0;
}

void SwarmPosefusionSolver::uavOdomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  Eigen::Quaterniond relative_orientation, odom_orientation;
  Eigen::Vector3d relative_position, odom_position;
  std::string numstr = msg->child_frame_id.substr(4);
  int now_id = std::stoi(numstr);

  ros::Time relative_time = uav_crepes_data_[now_id].relative_msg.header.stamp;
  if (fabs((msg->header.stamp - relative_time).toSec()) > 0.1)
  {
    return;
  }
  relative_orientation.x() = uav_crepes_data_[now_id].relative_msg.pose.pose.orientation.x; // in my frame look master
  relative_orientation.y() = uav_crepes_data_[now_id].relative_msg.pose.pose.orientation.y;
  relative_orientation.z() = uav_crepes_data_[now_id].relative_msg.pose.pose.orientation.z;
  relative_orientation.w() = uav_crepes_data_[now_id].relative_msg.pose.pose.orientation.w;
  relative_position.x() = uav_crepes_data_[now_id].relative_msg.pose.pose.position.x;
  relative_position.y() = uav_crepes_data_[now_id].relative_msg.pose.pose.position.y;
  relative_position.z() = uav_crepes_data_[now_id].relative_msg.pose.pose.position.z;

  odom_orientation.x() = msg->pose.pose.orientation.x;
  odom_orientation.y() = msg->pose.pose.orientation.y;
  odom_orientation.z() = msg->pose.pose.orientation.z;
  odom_orientation.w() = msg->pose.pose.orientation.w;
  odom_position.x() = msg->pose.pose.position.x;
  odom_position.y() = msg->pose.pose.position.y;
  odom_position.z() = msg->pose.pose.position.z;

  Eigen::Isometry3d T_ex1 = Eigen::Isometry3d::Identity();
  // T_ex1.pretranslate(Eigen::Vector3d(-0.0, 0.0, -0.11));// need correct in real

  Eigen::Isometry3d T_1 = Eigen::Isometry3d::Identity();
  T_1.rotate(relative_orientation.normalized().toRotationMatrix());
  T_1.pretranslate(relative_position);
  T_1 = T_1.inverse();

  Eigen::Isometry3d T_2 = Eigen::Isometry3d::Identity();
  T_2.rotate(odom_orientation.normalized().toRotationMatrix());
  T_2.pretranslate(odom_position);

  // Eigen::AngleAxisd v(M_PI / 2, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd v(0, Eigen::Vector3d::UnitZ());
  Eigen::Matrix3d rotMatrix = v.matrix();
  Eigen::Isometry3d T_ex2 = Eigen::Isometry3d::Identity();
  T_ex2.rotate(rotMatrix);
  // T_ex2.pretranslate(Eigen::Vector3d(0, 0.0, 0.08));

  Eigen::Isometry3d T_3 = T_2 * T_ex2 * T_1 * T_ex1;
  Eigen::Vector3d final_position = T_3.translation();                        // t_odom_curr_now 平移向量
  Eigen::Quaterniond final_orientation = Eigen::Quaterniond(T_3.rotation()); // 转为四元数

  nav_msgs::Odometry odom_fused;
  odom_fused.header.frame_id = frame_id_;
  odom_fused.child_frame_id = string("ugv_") + std::to_string(self_id_);
  odom_fused.header.stamp = relative_time;
  odom_fused.pose.pose.position.x = final_position.x();
  odom_fused.pose.pose.position.y = final_position.y();
  odom_fused.pose.pose.position.z = final_position.z();
  odom_fused.pose.pose.orientation.x = final_orientation.x();
  odom_fused.pose.pose.orientation.y = final_orientation.y();
  odom_fused.pose.pose.orientation.z = final_orientation.z();
  odom_fused.pose.pose.orientation.w = final_orientation.w();
  added_odom_pub_.publish(odom_fused);
  // ROS_WARN("Master add");
  if (!filter_init_)
  {
    VectorXd initx(6);
    // Eigen::Vector3d eulerAngle = final_orientation.matrix().eulerAngles(0,1,2);
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom_fused.pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //
    initx << final_position, roll, pitch, yaw;

    odom_filter_->filterInit(initx);
    cout << "have init" << endl;
    filter_init_ = true;
  }
  else
  {
    if (last_wheel_time_ - relative_time.toSec() < 0.11) // 0.05
    {
      if (!relative_flag_)
      {
        if (relative_cout_ < 0)
          relative_cout_++;
        else
        {
          relative_flag_ = true;
          relative_cout_ = 0;
        }
      }
      else
      {
        VectorXd measure(6);
        // Vector3d eulerAngle = final_orientation.matrix().eulerAngles(0,1,2);
        tf::Quaternion quat;
        tf::quaternionMsgToTF(odom_fused.pose.pose.orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //
        measure << final_position, roll, pitch, yaw;
        odom_filter_->kalmanUpdate(measure);
        last_uav_time_ = relative_time.toSec();
        // ROS_WARN("Master update");
      }
    }
    else
    {
      if (relative_flag_)
        relative_flag_ = false;
      ROS_ERROR("longtime master relative!!! %f", last_wheel_time_ - relative_time.toSec());
    }
  }
}

void SwarmPosefusionSolver::ugvOdomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  Eigen::Quaterniond relative_orientation, odom_orientation;
  Eigen::Vector3d relative_position, odom_position;
  std::string numstr = msg->child_frame_id.substr(4);
  int now_id = std::stoi(numstr);

  if (now_id == self_id_)
    return;

  ros::Time relative_time = ugv_crepes_data_[now_id].relative_msg.header.stamp;
  if (fabs((msg->header.stamp - relative_time).toSec()) > 0.11)
  {
    return;
  }

  relative_orientation.x() = ugv_crepes_data_[now_id].relative_msg.pose.pose.orientation.x; // in my frame look master
  relative_orientation.y() = ugv_crepes_data_[now_id].relative_msg.pose.pose.orientation.y;
  relative_orientation.z() = ugv_crepes_data_[now_id].relative_msg.pose.pose.orientation.z;
  relative_orientation.w() = ugv_crepes_data_[now_id].relative_msg.pose.pose.orientation.w;
  relative_position.x() = ugv_crepes_data_[now_id].relative_msg.pose.pose.position.x;
  relative_position.y() = ugv_crepes_data_[now_id].relative_msg.pose.pose.position.y;
  relative_position.z() = ugv_crepes_data_[now_id].relative_msg.pose.pose.position.z;

  odom_orientation.x() = msg->pose.pose.orientation.x;
  odom_orientation.y() = msg->pose.pose.orientation.y;
  odom_orientation.z() = msg->pose.pose.orientation.z;
  odom_orientation.w() = msg->pose.pose.orientation.w;
  odom_position.x() = msg->pose.pose.position.x;
  odom_position.y() = msg->pose.pose.position.y;
  odom_position.z() = msg->pose.pose.position.z;

  Eigen::Isometry3d T_ex1 = Eigen::Isometry3d::Identity();
  T_ex1.pretranslate(Eigen::Vector3d(-0.0, 0.0, -0.11)); // 0.12

  Eigen::Isometry3d T_1 = Eigen::Isometry3d::Identity();
  T_1.rotate(relative_orientation.normalized().toRotationMatrix());
  T_1.pretranslate(relative_position);
  T_1 = T_1.inverse();

  Eigen::Isometry3d T_2 = Eigen::Isometry3d::Identity();
  T_2.rotate(odom_orientation.normalized().toRotationMatrix());
  T_2.pretranslate(odom_position);

  Eigen::Isometry3d T_ex2 = Eigen::Isometry3d::Identity();
  T_ex2.pretranslate(Eigen::Vector3d(0.0, 0.0, 0.11)); // 0.12

  Eigen::Isometry3d T_3 = T_2 * T_ex2 * T_1 * T_ex1;
  Eigen::Vector3d final_position = T_3.translation();                        // t_odom_curr_now 平移向量
  Eigen::Quaterniond final_orientation = Eigen::Quaterniond(T_3.rotation()); // 转为四元数

  // 纯观测输出
  nav_msgs::Odometry odom_fused;
  odom_fused.header.frame_id = frame_id_;
  odom_fused.child_frame_id = string("ugv_") + std::to_string(self_id_);
  odom_fused.header.stamp = relative_time;
  odom_fused.pose.pose.position.x = final_position.x();
  odom_fused.pose.pose.position.y = final_position.y();
  odom_fused.pose.pose.position.z = final_position.z();
  odom_fused.pose.pose.orientation.x = final_orientation.x();
  odom_fused.pose.pose.orientation.y = final_orientation.y();
  odom_fused.pose.pose.orientation.z = final_orientation.z();
  odom_fused.pose.pose.orientation.w = final_orientation.w();
  added_odom_pub_.publish(odom_fused);
  // ROS_WARN("other add");
  if (!filter_init_)
  {
    VectorXd initx(6);
    // Eigen::Vector3d eulerAngle = final_orientation.matrix().eulerAngles(0,1,2);
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom_fused.pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //
    initx << final_position, roll, pitch, yaw;

    odom_filter_->filterInit(initx);
    cout << "have init" << endl;
    filter_init_ = true;
  }
  else
  {
    if (last_wheel_time_ - relative_time.toSec() < 0.11)
    {
      VectorXd measure(6);
      // Vector3d eulerAngle = final_orientation.matrix().eulerAngles(0,1,2);
      tf::Quaternion quat;
      tf::quaternionMsgToTF(odom_fused.pose.pose.orientation, quat);
      double roll, pitch, yaw;
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //
      measure << final_position, roll, pitch, yaw;
      odom_filter_->kalmanUpdate(measure);
      ROS_WARN("Other update");
    }
    else
    {
      ROS_ERROR("longtime other relative!!!");
    }
  }
}

void SwarmPosefusionSolver::wheelOdomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  if (filter_init_)
  {
    if (last_wheel_time_ == 0)
    {
      last_wheel_time_ = msg->header.stamp.toSec();
      twist_input_ << msg->twist.twist.linear.x,
          msg->twist.twist.linear.y,
          msg->twist.twist.linear.z,
          msg->twist.twist.angular.x,
          msg->twist.twist.angular.y,
          msg->twist.twist.angular.z;
      pubFilterOdom(msg->header.stamp);
      return;
    }
    else
    {
      double dt = msg->header.stamp.toSec() - last_wheel_time_;
      odom_filter_->kalmanPredict(twist_input_, dt);
      last_wheel_time_ = msg->header.stamp.toSec();
      twist_input_ << msg->twist.twist.linear.x,
          msg->twist.twist.linear.y,
          msg->twist.twist.linear.z,
          msg->twist.twist.angular.x,
          msg->twist.twist.angular.y,
          msg->twist.twist.angular.z;
      pubFilterOdom(msg->header.stamp);
    }
  }
  else if (self_init_)
  {
    VectorXd initx(6);
    // Eigen::Vector3d eulerAngle = final_orientation.matrix().eulerAngles(0,1,2);
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //
    initx << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, roll, pitch, yaw;

    odom_filter_->filterInit(initx);
    cout << "self init" << endl;
    filter_init_ = true;
  }
}

void SwarmPosefusionSolver::pubFilterOdom(ros::Time stamp)
{
  geometry_msgs::Quaternion q;
  nav_msgs::Odometry odom_filter;
  q = tf::createQuaternionMsgFromRollPitchYaw(odom_filter_->nowX(3), odom_filter_->nowX(4), odom_filter_->nowX(5));
  odom_filter.header.frame_id = frame_id_;
  odom_filter.child_frame_id = string("ugv_") + std::to_string(self_id_);
  odom_filter.header.stamp = stamp;
  odom_filter.pose.pose.position.x = odom_filter_->nowX(0);
  odom_filter.pose.pose.position.y = odom_filter_->nowX(1);
  odom_filter.pose.pose.position.z = 0.3;
  odom_filter.pose.pose.orientation = q;
  odom_filter.twist.twist.linear.x = twist_input_(0) * cos(odom_filter_->nowX(5)) - twist_input_(1) * sin(odom_filter_->nowX(5));
  odom_filter.twist.twist.linear.y = twist_input_(0) * sin(odom_filter_->nowX(5)) - twist_input_(1) * cos(odom_filter_->nowX(5));
  odom_filter.twist.twist.linear.z = 0;
  odom_filter.twist.twist.angular.x = twist_input_(3);
  odom_filter.twist.twist.angular.y = twist_input_(4);
  odom_filter.twist.twist.angular.z = twist_input_(5);

  Eigen::Map<Eigen::Matrix<double, 6, 6>>(odom_filter.pose.covariance.data()) = odom_filter_->nowP;

  // std::cout << "initP" << odom_filter_->nowP << std::endl;

  fused_odom_pub_.publish(odom_filter);

  // 纯预测输出
  q = tf::createQuaternionMsgFromRollPitchYaw(odom_filter_->testX(3), odom_filter_->testX(4), odom_filter_->testX(5));
  odom_filter.header.frame_id = frame_id_;
  odom_filter.child_frame_id = string("ugv_") + std::to_string(self_id_);
  odom_filter.header.stamp = stamp;
  odom_filter.pose.pose.position.x = odom_filter_->testX(0);
  odom_filter.pose.pose.position.y = odom_filter_->testX(1);
  odom_filter.pose.pose.position.z = odom_filter_->testX(2);
  odom_filter.pose.pose.orientation = q;
  predict_odom_pub_.publish(odom_filter);
}
