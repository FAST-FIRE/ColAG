#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <custom_msgs/map_info.h>
#include <custom_msgs/blind_info.h>

#include "swarm_bridge/reliable_bridge.hpp"

double delta_time_ = 0.0;

std::vector<int> id_list_;
std::vector<std::string> ip_list_;

ros::Subscriber map_sub_;
ros::Subscriber blind_sub_;
ros::Subscriber odom_sub_;
ros::Publisher map_pub_;
ros::Publisher blind_pub_;
ros::Publisher odom_pub_;

int self_id_;
int self_id_in_bridge_;
bool is_uav_;

int uav_num_;
int ugv_num_;

double map_freq_;
double odom_freq_;
double blind_freq_;

std::unique_ptr<ReliableBridge> bridge;

int remap_uav_id(int id)
{
  return id + ugv_num_;
}

int remap_frome_frameid_to_id(std::string frame_id)
{
  // frame_id formate is device_ + num;
  if (frame_id.length() < 5)
  {
    ROS_ERROR("Frame ID ERROR(1)");
    return -1;
  }
  std::string str_num = frame_id.substr(4);
  int id = std::atoi(str_num.c_str());
  return id;
}

template <typename T>
int send_to_all_ugv_except_me(std::string topic, T &msg)
{
  int err_code = 0;
  for (int i = 0; i < ugv_num_; ++i) // Only send to all devices.
  {
    if (i == self_id_in_bridge_) // skip myself
    {
      continue;
    }
    err_code += bridge->send_msg_to_one(i, topic, msg);
    if (err_code < 0)
    {
      ROS_ERROR("[Bridge] SEND ERROR %s !!", typeid(T).name());
    }
  }
  return err_code;
}

template <typename T>
int send_to_ugv_except_me(std::string topic, T &msg, int id)
{
  int err_code = 0;
  if (id >= ugv_num_) // Only send to all devices.
  {
    ROS_ERROR("[Bridge] ID ERROR %d !!", id);
  }
  if (id == self_id_in_bridge_) // skip myself
  {
    return err_code;
  }
  err_code += bridge->send_msg_to_one(id, topic, msg);
  if (err_code < 0)
  {
    ROS_ERROR("[Bridge] SEND ERROR %s !!", typeid(T).name());
  }
  return err_code;
}

template <typename T>
int send_to_all_uav_except_me(std::string topic, T &msg)
{
  int err_code = 0;
  for (int i = 0; i < uav_num_; ++i) // Only send to all groundstations.
  {
    int ind = remap_uav_id(i);
    if (ind == self_id_in_bridge_) // skip myself
    {
      continue;
    }
    err_code += bridge->send_msg_to_one(ind, topic, msg);
    if (err_code < 0)
    {
      ROS_ERROR("[Bridge] SEND ERROR %s !!", typeid(T).name());
    }
  }
  return err_code;
}

void register_callbak_to_all_uav(std::string topic_name, function<void(int, ros::SerializedMessage &)> callback)
{
  for (int i = 0; i < uav_num_; ++i)
  {
    int ind = remap_uav_id(i);
    if (ind == self_id_in_bridge_) // skip myself
    {
      continue;
    }
    bridge->register_callback(ind, topic_name, callback);
  }
}

void register_callbak_to_all_ugv(std::string topic_name, function<void(int, ros::SerializedMessage &)> callback)
{
  for (int i = 0; i < ugv_num_; ++i)
  {
    if (i == self_id_in_bridge_) // skip myself
    {
      continue;
    }
    bridge->register_callback(i, topic_name, callback);
  }
}

void delta_time_callback(const std_msgs::Float64ConstPtr &delta_time_msg)
{
  delta_time_ = delta_time_msg->data;
}

void map_sub_tcp_callback(const custom_msgs::map_infoConstPtr &msg)
{
  auto map_msg = *msg;
  double t = map_msg.header.stamp.toSec();
  map_msg.header.stamp = ros::Time().fromSec(t - delta_time_);
  map_msg.header.frame_id = std::string("uav_") + std::to_string(self_id_);
  static ros::Time t_last;
  ros::Time t_now = ros::Time::now();
  if ((t_now - t_last).toSec() < 1.0 / map_freq_)
  {
    return;
  }
  t_last = t_now;
  // send_to_all_ugv_except_me("/map", map_msg);
  send_to_ugv_except_me("/map", map_msg, msg->id_to);
}

void blind_sub_tcp_callback(const custom_msgs::blind_infoConstPtr &msg)
{
  auto blind_msg = *msg;
  double t = blind_msg.header.stamp.toSec();
  blind_msg.header.stamp = ros::Time().fromSec(t - delta_time_);
  blind_msg.header.frame_id = std::string("ugv_") + std::to_string(self_id_);
  static ros::Time t_last;
  ros::Time t_now = ros::Time::now();
  if ((t_now - t_last).toSec() < 1.0 / blind_freq_)
  {
    return;
  }
  t_last = t_now;
  send_to_all_uav_except_me("/blind", blind_msg);
}

void odom_sub_tcp_callback(const nav_msgs::OdometryConstPtr &msg)
{
  nav_msgs::Odometry odom_msg = *msg;
  double t = odom_msg.header.stamp.toSec();
  odom_msg.header.stamp = ros::Time().fromSec(t - delta_time_);
  static ros::Time t_last;
  ros::Time t_now = ros::Time::now();
  if ((t_now - t_last).toSec() < 1.0 / odom_freq_)
  {
    return;
  }
  t_last = t_now;

  if (is_uav_)
  {
    odom_msg.header.frame_id = std::string("uav_") + std::to_string(self_id_);
    send_to_all_ugv_except_me("/odom", odom_msg);
  }
  else
  {
    odom_msg.header.frame_id = std::string("ugv_") + std::to_string(self_id_);
    send_to_all_uav_except_me("/odom", odom_msg);
  }
}

// Here is callback when the brodge received the data from others.
void map_bridge_callback(int ID, ros::SerializedMessage &m)
{
  custom_msgs::map_info map_msg;
  ros::serialization::deserializeMessage(m, map_msg);
  map_msg.header.stamp = ros::Time().fromSec(map_msg.header.stamp.toSec() + delta_time_);
  int recv_device_id = remap_frome_frameid_to_id(map_msg.header.frame_id);
  if (recv_device_id < 0 || recv_device_id >= ugv_num_)
  {
    ROS_ERROR("Frame ID ERROE(m)");
    return;
  }
  map_msg.header.frame_id = "world";
  map_pub_.publish(map_msg);
}

// Here is callback when the brodge received the data from others.
void blind_bridge_callback(int ID, ros::SerializedMessage &m)
{
  custom_msgs::blind_info blind_msg;
  ros::serialization::deserializeMessage(m, blind_msg);
  blind_msg.header.stamp = ros::Time().fromSec(blind_msg.header.stamp.toSec() + delta_time_);
  int recv_device_id = remap_frome_frameid_to_id(blind_msg.header.frame_id);
  if (recv_device_id < 0 || recv_device_id >= ugv_num_)
  {
    ROS_ERROR("Frame ID ERROE(m)");
    return;
  }
  blind_msg.header.frame_id = "world";
  blind_pub_.publish(blind_msg);
}

// Here is callback when the brodge received the data from others.
void odom_bridge_callback(int ID, ros::SerializedMessage &m)
{
  // 这里的ID和下面的recv_device_id是一样的，抓要是由于每个线程监视一个ip，每个ip和设备是绑定的。
  nav_msgs::Odometry odom_msg;
  ros::serialization::deserializeMessage(m, odom_msg);
  odom_msg.header.stamp = ros::Time().fromSec(odom_msg.header.stamp.toSec() + delta_time_);
  int recv_device_id = remap_frome_frameid_to_id(odom_msg.header.frame_id);
  if (is_uav_)
  {
    if (recv_device_id < 0 || recv_device_id >= ugv_num_)
    {
      ROS_ERROR("Frame ID ERROE(G)");
      return;
    }
    odom_msg.header.frame_id = "world";
    odom_msg.child_frame_id = std::string("ugv_") + std::to_string(recv_device_id);
    odom_pub_.publish(odom_msg);
  }
  else
  {
    if (recv_device_id < 0 || recv_device_id >= uav_num_)
    {
      ROS_ERROR("Frame ID ERROE(A)");
      return;
    }
    odom_msg.header.frame_id = "world";
    odom_msg.child_frame_id = std::string("uav_") + std::to_string(recv_device_id);
    odom_pub_.publish(odom_msg);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_bridge");
  ros::NodeHandle nh("~");

  nh.param("self_id", self_id_, -1);
  nh.param("is_uav", is_uav_, false);
  nh.param("map_freq", map_freq_, 2.0);
  nh.param("blind_freq", blind_freq_, 20.0);
  nh.param("odom_freq", odom_freq_, 20.0);

  nh.param("uav_num", uav_num_, 0);
  nh.param("ugv_num", ugv_num_, 0);

  id_list_.resize(ugv_num_ + uav_num_);
  ip_list_.resize(ugv_num_ + uav_num_);

  for (int i = 0; i < ugv_num_ + uav_num_; ++i)
  {
    nh.param((i < ugv_num_ ? "ugv_ip_" + to_string(i) : "uav_ip_" + to_string(i - ugv_num_)), ip_list_[i], std::string("127.0.0.1"));
    id_list_[i] = i;
  }
  self_id_in_bridge_ = self_id_;
  if (is_uav_)
  {
    self_id_in_bridge_ = remap_uav_id(self_id_);
  }
  // the uav ID = self ID + ugv_num_
  if (self_id_in_bridge_ < 0 || self_id_in_bridge_ > 99)
  {
    ROS_WARN("[swarm bridge] Wrong self_id!");
    exit(EXIT_FAILURE);
  }

  ros::Subscriber delta_time_sub = nh.subscribe("delta_time_topic", 1, delta_time_callback);

  if (is_uav_)
  {
    map_sub_ = nh.subscribe<custom_msgs::map_info>("map_info", 10, map_sub_tcp_callback, ros::TransportHints().tcpNoDelay());
    blind_pub_ = nh.advertise<custom_msgs::blind_info>("blind_info2", 1);

    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 10, odom_sub_tcp_callback, ros::TransportHints().tcpNoDelay());
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom2", 1);
    // initalize the bridge
    bridge.reset(new ReliableBridge(self_id_in_bridge_, ip_list_, id_list_, 100000));
    register_callbak_to_all_ugv("/odom", odom_bridge_callback);
    register_callbak_to_all_ugv("/blind", blind_bridge_callback);
  }
  else
  {
    blind_sub_ = nh.subscribe<custom_msgs::blind_info>("blind_info", 10, blind_sub_tcp_callback, ros::TransportHints().tcpNoDelay());
    map_pub_ = nh.advertise<custom_msgs::map_info>("map_info2", 1);

    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 10, odom_sub_tcp_callback, ros::TransportHints().tcpNoDelay());
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom2", 1);
    // initalize the bridge
    bridge.reset(new ReliableBridge(self_id_in_bridge_, ip_list_, id_list_, 100000));
    register_callbak_to_all_uav("/odom", odom_bridge_callback);
    register_callbak_to_all_uav("/map", map_bridge_callback);
  }

  ros::spin();

  bridge->StopThread();
  return 0;
}