#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "traj_utils/Bspline.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <math.h>

ros::Publisher pos_cmd_pub,a;

// quadrotor_msgs::PositionCommand cmd;
geometry_msgs::Twist cmd;
double gain, wheelbase,lfc;

using ego_planner::UniformBspline;

bool receive_traj_ = false;
bool has_odom = false;
vector<UniformBspline> traj_;
double traj_duration_;
ros::Time start_time_, true_start_time_;
int traj_id_;

double time_cost;
// yaw control
double t_cur;
bool back_flag = false, end_flag = false;
double time_forward_;
double last_yaw_, last_yaw_dot_;
double now_yaw, now_v;
Eigen::Vector3d now_pos;
double max_steer, v_max_, max_kappa;

void rcvOdomCallBack(nav_msgs::OdometryPtr msg)
{
  has_odom = true;
  now_pos[0] = msg->pose.pose.position.x;
  now_pos[1] = msg->pose.pose.position.y;
  now_pos[2] = msg->pose.pose.position.z;
  Eigen::Quaterniond q(msg->pose.pose.orientation.w,
			msg->pose.pose.orientation.x,
			msg->pose.pose.orientation.y,
			msg->pose.pose.orientation.z);
  Eigen::Matrix3d R(q);
  Eigen::Vector2d lvel(msg->twist.twist.linear.x,msg->twist.twist.linear.y);
  last_yaw_ = now_yaw;
  now_yaw = atan2(R.col(0)[1],R.col(0)[0]);
  now_v = lvel.norm();
}

void bsplineCallback(traj_utils::BsplineConstPtr msg)
{
  // parse pos traj

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

  Eigen::VectorXd knots(msg->knots.size());
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  start_time_ = true_start_time_ = msg->start_time;
  time_cost = 0;

  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());

  traj_duration_ = traj_[0].getTimeSum();
  receive_traj_ = true;
  end_flag = false;
  back_flag = false;
}

void cmdCallback(const ros::TimerEvent &e)
{
  /* no publishing before receive traj_ */
  if ((!receive_traj_) || (!has_odom))
    return;
  static ros::Time time_last;
  ros::Time time_now = ros::Time::now();
  double speed=0.0, steer=0.0;
  if (!back_flag)
  {
    t_cur = (time_now - start_time_).toSec(); //- time_cost
  }
  else
  {
    time_cost += (time_now - time_last).toSec() ;
  }
   time_last = ros::Time::now();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero());
  if((now_pos -traj_[0].evaluateDeBoorT(traj_duration_)).norm() < 0.01)
  {
    speed = 0.0;
    steer=0.0;
  }
  else
  {
    if (t_cur < traj_duration_ && t_cur >= 0.0)
    {
      vel = traj_[1].evaluateDeBoorT(t_cur);
      pos = traj_[0].evaluateDeBoorT(t_cur);
      vel(2) = 0;
      pos(2) = 0;
      double v_temp = vel .norm();
      Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - now_pos : traj_[0].evaluateDeBoorT(traj_duration_) - now_pos;

      if(dir.dot(traj_[1].evaluateDeBoorT(t_cur))<0)//超前了//maybe also match the delay
      {
        speed = 0.0;
        steer = 0.0;
      }
      else
      {
        double yaw_temp = dir.norm() > 0.01 ? atan2(dir(1), dir(0)) : last_yaw_;//距离近的话就用上一次的角度
        if (yaw_temp - now_yaw > M_PI)
        {
          if (yaw_temp - now_yaw - 2 * M_PI < -M_PI_2)
          {
            back_flag = true;
            steer = -max_steer;
            cout<<"max_yawrate1 "<<yaw_temp<<" "<<now_yaw<<endl;
          }
          else
          {
            back_flag = false;
            speed = now_v + gain*(v_temp - now_v);
            steer = -wheelbase*(2*M_PI - yaw_temp + now_yaw)/lfc;
          }
        }
        else if (yaw_temp - now_yaw < -M_PI)
        {
          if (yaw_temp - now_yaw+ 2 * M_PI > M_PI_2)
          {
            back_flag = true;
            steer = max_steer;
            cout<<"max_yawrate2 "<<yaw_temp<<" "<<now_yaw<<endl;
          }
          else
          {
            back_flag = false;
            speed = now_v + gain*(v_temp - now_v);
            steer = wheelbase*(2*M_PI + yaw_temp - now_yaw)/lfc;
          }
        }
        else
        {
          if (yaw_temp - now_yaw < -M_PI_2)
          {
            back_flag = true;
            steer = -max_steer;
            cout<<"max_yawrate3 "<<yaw_temp<<" "<<now_yaw<<endl;
          }
          else if (yaw_temp - now_yaw > M_PI_2)
          {
            back_flag = true;
            steer = max_steer;
            cout<<"max_yawrate4 "<<yaw_temp<<" "<<now_yaw<<endl;
          }
          else
          {
            back_flag = false;
            speed = now_v + gain*(v_temp - now_v);
            steer = wheelbase*(yaw_temp - now_yaw)/lfc;
          }
        }
      }

    }
    else if (t_cur >= traj_duration_)
    {
      /* hover when finish traj_ */
      if((traj_[0].evaluateDeBoorT(traj_duration_)-now_pos).dot(traj_[1].evaluateDeBoorT(traj_duration_-time_forward_))>0.1)//还在朝者这个方向走
      {
        speed = traj_[1].evaluateDeBoorT(traj_duration_).norm();
        //cout <<"speed still" <<speed<< endl;
      }
      else 
      {
        speed = 0;
      }
      steer = 0;
      end_flag = false;
    }
    else
    {
      cout << "[Traj server]: invalid time." << endl;
    }
  }


  cmd.linear.x = speed;
  cmd.linear.y = 0;
  cmd.linear.z = 0;
  cmd.angular.x = 0;
  cmd.angular.y = 0;
  cmd.angular.z = steer;

  pos_cmd_pub.publish(cmd);

  // debug visualization
  // nav_msgs::Odometry sphere;
  // sphere.header.stamp = ros::Time::now();
  // sphere.header.frame_id = "world";
  // sphere.pose.pose.position.x = traj_[0].evaluateDeBoorT(t_temp)[0];
  // sphere.pose.pose.position.y = traj_[0].evaluateDeBoorT(t_temp)[1];
  // sphere.pose.pose.position.z = traj_[0].evaluateDeBoorT(t_temp)[2];
  // sphere.pose.pose.orientation.w = 1;
  // sphere.pose.pose.orientation.x = 0;
  // sphere.pose.pose.orientation.y = 0;
  // sphere.pose.pose.orientation.z = 0;

  //a.publish(sphere);

}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber bspline_sub = nh.subscribe("planning/bspline", 10, bsplineCallback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber odom_sub_ = nh.subscribe( "odom", 1, rcvOdomCallBack, ros::TransportHints().tcpNoDelay());

  pos_cmd_pub = nh.advertise<geometry_msgs::Twist>("/position_cmd", 50);
  //a = nh.advertise<nav_msgs::Odometry>( "/track_point", 1 );
  // pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);]

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

  /* control parameter */
  nh.param("traj_server/time_forward", time_forward_, -1.0);
  nh.param("traj_server/max_steer", max_steer, -1.0);
  nh.param("traj_server/v_max", v_max_, -1.0);
  nh.param("traj_server/gain", gain, -1.0);
  nh.param("traj_server/wheelbase", wheelbase, -1.0);
  nh.param("traj_server/forward_distance",lfc,-1.0);
  max_kappa = tan(max_steer)/lfc;

  last_yaw_ = 0.0;
  now_yaw = now_v = 0.0;

  ros::Duration(1.0).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}