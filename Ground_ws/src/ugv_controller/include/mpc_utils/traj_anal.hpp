#pragma once

#include "mpc/Polynome.h"
#include "mpc_utils/minco.hpp"
#include <ros/ros.h>

template <typename T>
T lerp(const T &x0, const double t0, const T &x1, const double t1,
       const double t) {
  if (std::abs(t1 - t0) <= 1.0e-6) {
    // AERROR << "input time difference is too small";
    return x0;
  }
  const double r = (t - t0) / (t1 - t0);
  const T x = x0 + r * (x1 - x0);
  return x;
}

inline double NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

class TrajPoint
{
public:
    double x = 0;
    double y = 0;
    double v = 0; 
    double a = 0;
    double theta = 0;
    double w = 0;
};

class TrajAnalyzer{
private:
    mpc_utils::MinJerkOpt jerk_opt;
    mpc_utils::Trajectory minco_traj;
    ros::Time start_time;
    double traj_duration;

public:

    bool back_track = false;
    bool at_goal = false;

    TrajAnalyzer() {}

    void setTraj(mpc::PolynomeConstPtr msg)
    {
      start_time = msg->start_time;

      Eigen::MatrixXd posP(3, msg->pos_pts.size() - 2);
      Eigen::VectorXd T(msg->t_pts.size());
      Eigen::MatrixXd initS, tailS;

      for (int i = 1; i < (int)msg->pos_pts.size() - 1; i++)
      {
        posP(0, i - 1) = msg->pos_pts[i].x;
        posP(1, i - 1) = msg->pos_pts[i].y;
        posP(2, i - 1) = msg->pos_pts[i].z;
      }
      for (int i = 0; i < (int)msg->t_pts.size(); i++)
      {
        T(i) = msg->t_pts[i];
      }

      initS.setZero(3, 3);
      tailS.setZero(3, 3);
      initS.col(0) = Eigen::Vector3d(msg->pos_pts[0].x, msg->pos_pts[0].y, msg->pos_pts[0].z);
      initS.col(1) = Eigen::Vector3d(msg->init_v.x, msg->init_v.y, msg->init_v.z);
      initS.col(2) = Eigen::Vector3d(msg->init_a.x, msg->init_a.y, msg->init_a.z);
      tailS.col(0) = Eigen::Vector3d(msg->pos_pts.back().x, msg->pos_pts.back().y, msg->pos_pts.back().z);
      tailS.col(1) = Eigen::Vector3d::Zero();
      tailS.col(2) = Eigen::Vector3d::Zero();
      jerk_opt.reset(initS, msg->pos_pts.size() - 1);
      jerk_opt.generate(posP, tailS, T);
      minco_traj = jerk_opt.getTraj();
      traj_duration = minco_traj.getTotalDuration();
    }

    std::vector<TrajPoint> getRefPoints(const int T, double dt)
    {
      std::vector<TrajPoint> P;
      P.clear();
      TrajPoint tp;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - start_time).toSec();
      int j=0;

      if (t_cur > traj_duration + 1.0)
      {
        at_goal = true;
        return P;
      }
      else
      {
        at_goal = false;
      }

      for (double t=t_cur+dt; j<T; j++, t+=dt)
      {
        double temp = t;
        if (temp <= traj_duration)
        {
          Eigen::Vector3d po = minco_traj.getPos(temp);
          Eigen::Vector3d pr = minco_traj.getPos(temp-dt);

          tp.v = minco_traj.getVel(temp).norm();
          tp.x = po[0];
          tp.y = po[1];
          tp.a = minco_traj.getAcc(temp).norm();
          tp.theta = atan2(pr[1]-po[1], pr[0]-po[0]);
          tp.w = 0;
          P.push_back(tp);
        }
        else
        {
          Eigen::Vector3d po = minco_traj.getPos(traj_duration);
          Eigen::Vector3d pr = minco_traj.getPos(traj_duration-dt);

          tp.v = minco_traj.getVel(traj_duration).norm();
          tp.x = po[0];
          tp.y = po[1];
          tp.a = minco_traj.getAcc(traj_duration).norm();
          tp.theta = atan2(pr[1]-po[1], pr[0]-po[0]);
          tp.w = 0;
          P.push_back(tp);
        }
      }

      return P;
    }

   
    ~TrajAnalyzer() {}
};