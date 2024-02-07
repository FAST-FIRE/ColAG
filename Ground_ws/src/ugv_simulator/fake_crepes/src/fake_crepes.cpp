#include <iostream>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <random>

bool have_noise;
double detect_threshold_;
Eigen::Matrix4d getTransformMatrix(nav_msgs::Odometry odom)
{
  Eigen::Matrix4d T;
  T.setIdentity();
  T.block(0, 0, 3, 3) = Eigen::Quaterniond(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z).toRotationMatrix();
  T.block(0, 3, 3, 1) = Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
  return T;
}

nav_msgs::Odometry getOdometry(Eigen::Matrix4d T)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<> d(0, 0.2);
  nav_msgs::Odometry odom;
  if (have_noise)
  {
    odom.pose.pose.position.x = T(0, 3) + d(gen);
    odom.pose.pose.position.y = T(1, 3) + d(gen);
    odom.pose.pose.position.z = T(2, 3) + d(gen);
    Eigen::Quaterniond q;
    q = Eigen::Matrix3d(T.block(0, 0, 3, 3));
    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
  }
  else
  {
    odom.pose.pose.position.x = T(0, 3);
    odom.pose.pose.position.y = T(1, 3);
    odom.pose.pose.position.z = T(2, 3);
    Eigen::Quaterniond q;
    q = Eigen::Matrix3d(T.block(0, 0, 3, 3));
    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
  }

  return odom;
}

Eigen::Matrix4d self_odom = Eigen::Matrix4d::Identity();
ros::Time self_odom_stamp;
void selfOdomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  self_odom = getTransformMatrix(*msg);
  self_odom_stamp = msg->header.stamp;
}

class Crepes
{
public:
  ros::Publisher crepes_pub;
  ros::Subscriber odom_sub;
  void odomCallback(const nav_msgs::OdometryConstPtr &msg);
};

void Crepes::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  // std::cout<<"in it"<<std::endl;
  Eigen::Matrix4d T = getTransformMatrix(*msg);
  Eigen::Matrix4d T_crepes = self_odom.inverse() * T;
  double distance = T_crepes.block(0, 3, 3, 1).norm();
  if (distance > detect_threshold_)
  {
    return;
  }
  if (abs(self_odom_stamp.toSec()-msg->header.stamp.toSec())>0.1)
  {
    return;
  }
  nav_msgs::Odometry crepes_odom = getOdometry(T_crepes);
  crepes_odom.header = msg->header;
  crepes_odom.child_frame_id = msg->child_frame_id;
  crepes_pub.publish(crepes_odom);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_crepes");
  ros::NodeHandle nh("~");

  int self_id;
  int uav_num, ugv_num;
  std::string ugv_odom_topic;
  std::string uav_odom_topic;

  nh.param("have_noise", have_noise, true);
  nh.param("self_id", self_id, -1);
  nh.param("uav_num", uav_num, 0);
  nh.param("ugv_num", ugv_num, 0);
  nh.param("detect_threshold", detect_threshold_, 5.0);
  nh.param("ugv_odom_topic", ugv_odom_topic, std::string("null"));
  nh.param("uav_odom_topic", uav_odom_topic, std::string("null"));

  std::vector<Crepes> uav_crepes_data(uav_num);
  std::vector<Crepes> ugv_crepes_data(ugv_num);

  ros::Subscriber self_odom_sub = nh.subscribe("self_odom", 1, selfOdomCallback, ros::TransportHints().tcpNoDelay());
  for (int i = 0; i < uav_num; i++)
  {
    std::string sub_topic_name = std::string("/uav_") + std::to_string(i) + std::string("/") + uav_odom_topic;
    uav_crepes_data[i].odom_sub = nh.subscribe(sub_topic_name, 1, &Crepes::odomCallback, &uav_crepes_data[i], ros::TransportHints().tcpNoDelay());
    uav_crepes_data[i].crepes_pub = nh.advertise<nav_msgs::Odometry>(std::string("uav_") + std::to_string(i) + std::string("_odom"), 1);
  }
  for (int i = 0; i < ugv_num; i++)
  {
    if (i == self_id)
    {
      continue;
    }
    std::string sub_topic_name = std::string("/ugv_") + std::to_string(i) + std::string("/") + ugv_odom_topic;
    ugv_crepes_data[i].odom_sub = nh.subscribe(sub_topic_name, 1, &Crepes::odomCallback, &ugv_crepes_data[i], ros::TransportHints().tcpNoDelay());
    ugv_crepes_data[i].crepes_pub = nh.advertise<nav_msgs::Odometry>(std::string("ugv_") + std::to_string(i) + std::string("_odom"), 1);
  }

  ros::spin();
  return 0;
}
