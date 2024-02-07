#include <iostream>
#include <stdlib.h>
#include <Eigen/Eigen>
// #include "ceres/ceres.h"

#include <nav_msgs/Odometry.h>
class DataManager
{
private:
  int self_id;

public:
  int robo_id;
  ros::Subscriber relative_odom_sub_;

  nav_msgs::Odometry relative_msg;
  std::vector<double> relative_msgs_new_timestamps;
  std::vector<double> relative_msgs_delete_timestamps;
  nav_msgs::Odometry odom_msg;
  std::vector<double> odom_msgs_new_timestamps;
  std::vector<double> odom_msgs_delete_timestamps;

  double newest_timestamp = 0;
  bool have_rl = false, have_ms = false;

  DataManager()
  {
  }
  DataManager(int id)
  {
    self_id = id;
  }
  void init(const int roboid, int self_id)
  {
    robo_id = roboid;
    self_id = self_id;
  }

  void clear_buffer()
  {
    relative_msgs_new_timestamps.clear();
    relative_msgs_delete_timestamps.clear();
    odom_msgs_new_timestamps.clear();
    odom_msgs_delete_timestamps.clear();
  }
  void relativeOdomCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    relative_msg = *msg;
    have_ms = true;
    // std::cout<<"have relative"<< std::endl;
  }
};

class PoseEKF
{
private:
  Eigen::Matrix<double, 3, 3> Q;
  Eigen::Matrix<double, 6, 6> R;
  Eigen::Matrix<double, 6, 6> F;
  Eigen::Matrix<double, 6, 6> H;

public:
  Eigen::Matrix<double, 6, 1> preX;
  Eigen::Matrix<double, 6, 1> nowX;
  Eigen::Matrix<double, 6, 1> testX;

  Eigen::Matrix<double, 6, 6> preP;
  Eigen::Matrix<double, 6, 6> nowP;
  double safe_theta_ = M_PI / 4;
  int change_cout_[4] = {0};

  PoseEKF(double q, double r)
  {
    Q << 0.0336, 0, 0,
        0, 0.0,  0, 
        0, 0,  0.02924;
    std::cout << "Q" << Q << std::endl;
    R << 0.2, 0, 0, 0, 0, 0,
        0, 0.2, 0, 0, 0, 0,
        0, 0, 1000.0, 0, 0, 0,
        0, 0, 0, 1000.0, 0, 0,
        0, 0, 0, 0, 1000.0, 0,
        0, 0, 0, 0, 0, 0.05;
    // R.setIdentity(6,6);
    // R = R*r;

    std::cout << "R" << R << std::endl;
    F.setIdentity(6, 6);
    H.setIdentity(6, 6);
  }
  void filterInit(const Eigen::VectorXd initX)
  {
    nowX = initX;
    testX = initX;

    nowP.setIdentity(6, 6);
    nowP = nowP * 10000;
    // std::cout << "initP" << nowP << std::endl;
  }
  void kalmanPredict(const Eigen::VectorXd input, double dt)
  {
    preX = nowX;
    preP = nowP;

    // cout<<"input"<<input<<endl;
    nowX(0) = preX(0) + (input(0) * cos(preX(5)) - input(1) * sin(preX(5))) * dt;
    nowX(1) = preX(1) + (input(1) * cos(preX(5)) + input(0) * sin(preX(5))) * dt;
    nowX(2) = preX(2) + input(2) * dt;
    nowX(3) = preX(3) + input(3) * dt;
    nowX(4) = preX(4) + input(4) * dt;
    nowX(5) = preX(5) + input(5) * dt;
    testX(0) = testX(0) + (input(0) * cos(testX(5)) - input(1) * sin(testX(5))) * dt;
    testX(1) = testX(1) + (input(1) * cos(testX(5)) + input(0) * sin(testX(5))) * dt;
    testX(2) = testX(2) + input(2) * dt;
    testX(3) = testX(3) + input(3) * dt;
    testX(4) = testX(4) + input(4) * dt;
    testX(5) = testX(5) + input(5) * dt;

    // cout<<"now"<<nowX<<endl;
    for (int i = 3; i <= 5; i++)
    {
      while ((nowX(i)) > M_PI)
        nowX(i) -= 2 * M_PI;
      while ((nowX(i)) < -M_PI)
        nowX(i) += 2 * M_PI;
    }

    for (int i = 3; i <= 5; i++)
    {
      while ((testX(i)) > M_PI)
        testX(i) -= 2 * M_PI;
      while ((testX(i)) < -M_PI)
        testX(i) += 2 * M_PI;
    }

    F(0, 5) = dt * (-input(0) * sin(preX(5)) - input(1) * cos(preX(5)));
    F(1, 5) = dt * (-input(1) * sin(preX(5)) + input(0) * cos(preX(5)));
    // K()

    Eigen::MatrixXd B;
    B.resize(6,3);
    B << dt*cos(preX(5)), -dt*sin(preX(5)), 0,
         dt*sin(preX(5)), dt*cos(preX(5)),  0,
         0,               0,                0,
         0,               0,                0,
         0,               0,                0,
         0,               0,                dt;
    nowP = F * preP * F.transpose() + B * Q * B.transpose();
  }

  void kalmanUpdate(const Eigen::VectorXd measure)
  {
    Eigen::MatrixXd K = nowP * H.transpose() * (H * nowP * H.transpose() + R).inverse();
    Eigen::VectorXd Z = H * nowX;
    Eigen::VectorXd errorZ = (measure - Z);

    // 解决飘的很慢的情况 the measurement is slowly change
    Eigen::Vector3d change1 = (nowX - preX).segment<3>(0);
    Eigen::Vector3d change2 = (measure - preX).segment<3>(0);
    double change_theta = acosIn2PI(change1, change2);
    if (change1.norm() > 0.02)
    {
      if (change_theta > safe_theta_ && change_theta < M_PI - safe_theta_)
      {
        change_cout_[0] = 0;
        change_cout_[1]++;
        change_cout_[2] = 0;
        change_cout_[3] = 0;
      }
      else if (change_theta > M_PI - safe_theta_ && change_theta < -M_PI + safe_theta_)
      {
        change_cout_[0] = 0;
        change_cout_[1] = 0;
        change_cout_[2]++;
        change_cout_[3] = 0;
      }
      else if (change_theta > -M_PI + safe_theta_ && change_theta < -safe_theta_)
      {
        change_cout_[0] = 0;
        change_cout_[1] = 0;
        change_cout_[2] = 0;
        change_cout_[3]++;
      }
      else
      {
        change_cout_[0] = 0;
        change_cout_[1] = 0;
        change_cout_[0] = 0;
        change_cout_[3] = 0;
      }
      if (change_cout_[0] + change_cout_[1] + change_cout_[2] + change_cout_[3] > 50)
      {
        ROS_ERROR("May be have drift!!!");
        K.block<6, 6>(0, 0) = Eigen::MatrixXd::Zero(6, 6);
      }
    }
    // 可以解决飘的很突然的情况 the measurement is suddenly chenge
    if (errorZ.segment<3>(0).norm() > 1.5)
    {
      ROS_ERROR("Position diff too much between measurement and prediction!!!");
      K.block<6, 6>(0, 0) = Eigen::MatrixXd::Zero(6, 6);
    }
    else if (abs(errorZ(3)) > 3 || abs(errorZ(4)) > 3 || abs(errorZ(5)) > 3)
    {
      ROS_ERROR("Angle diff too much between measurement and prediction!!!");
      K.block<3, 6>(3, 0) = Eigen::MatrixXd::Zero(3, 6);
    }

    nowX = nowX + K * errorZ;
    nowP = (Eigen::MatrixXd::Identity(6, 6) - K * H) * nowP;
    // cout <<nowP<<endl;
  }

private:
  double acosIn2PI(Eigen::Vector3d change1, Eigen::Vector3d change2)
  {
    double theta;
    theta = acos(change1.dot(change2) / (change2.norm() * change2.norm()));
    if ((change1.cross(change2)).z() < 0)
    {
      theta = -theta;
    }
    return theta;
  }

public:
  typedef std::unique_ptr<PoseEKF> Ptr;
};