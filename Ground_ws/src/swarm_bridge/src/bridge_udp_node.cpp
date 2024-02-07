#include <ros/ros.h>
#include <boost/thread.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>

#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <traj_utils/MultiBsplines.h>
#include <traj_utils/Bspline.h>

#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

#define UDP_PORT 8081
#define BUF_LEN 1048576    // 1MB
#define BUF_LEN_SHORT 1024 // 1KB

using namespace std;

double delta_time_ = 0.0;


int udp_server_fd_, udp_send_fd_;

string udp_ip_;

int self_id_ = -1;

double odom_broadcast_freq_;

char udp_recv_buf_[BUF_LEN], udp_send_buf_[BUF_LEN];
struct sockaddr_in addr_udp_send_;

ros::Subscriber my_odom_sub_, my_traj_sub_;
ros::Publisher other_odom_pub_, other_traj_pub_;

enum MESSAGE_TYPE
{
  ODOM = 100,
  TRAJ,
};

void delta_time_callback(const std_msgs::Float64ConstPtr &delta_time_msg)
{
  delta_time_ = delta_time_msg->data;
}

int init_broadcast(const char *ip, const int port)
{
  int fd;

  if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) <= 0)
  {
    ROS_ERROR("[bridge_node]Socket sender creation error!");
    exit(EXIT_FAILURE);
  }

  int so_broadcast = 1;
  if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &so_broadcast, sizeof(so_broadcast)) < 0)
  {
    ROS_ERROR("Error in setting Broadcast option");
    exit(EXIT_FAILURE);
  }

  addr_udp_send_.sin_family = AF_INET;
  addr_udp_send_.sin_port = htons(port);

  if (inet_pton(AF_INET, ip, &addr_udp_send_.sin_addr) <= 0)
  {
    ROS_ERROR("\nInvalid address/ Address not supported \n");
    return -1;
  }

  return fd;
}

int udp_bind_to_port(const int port, int &server_fd)
{
  struct sockaddr_in address;
  int opt = 1;

  // Creating socket file descriptor
  if ((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) == 0)
  {
    ROS_ERROR("socket failed");
    exit(EXIT_FAILURE);
  }

  // Forcefully attaching socket to the port
  if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
                 &opt, sizeof(opt)))
  {
    ROS_ERROR("setsockopt");
    exit(EXIT_FAILURE);
  }
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(port);

  // Forcefully attaching socket to the port
  if (bind(server_fd, (struct sockaddr *)&address,
           sizeof(address)) < 0)
  {
    ROS_ERROR("bind failed");
    exit(EXIT_FAILURE);
  }

  return server_fd;
}

template <typename T>
int serializeTopic(const MESSAGE_TYPE msg_type, const T &msg)
{
  auto ptr = (uint8_t *)(udp_send_buf_);

  *((MESSAGE_TYPE *)ptr) = msg_type;
  ptr += sizeof(MESSAGE_TYPE);

  namespace ser = ros::serialization;
  uint32_t msg_size = ser::serializationLength(msg);

  *((uint32_t *)ptr) = msg_size;
  ptr += sizeof(uint32_t);

  ser::OStream stream(ptr, msg_size);
  ser::serialize(stream, msg);

  return msg_size + sizeof(MESSAGE_TYPE) + sizeof(uint32_t);
}

template <typename T>
int deserializeTopic(T &msg)
{
  auto ptr = (uint8_t *)(udp_recv_buf_ + sizeof(MESSAGE_TYPE));

  uint32_t msg_size = *((uint32_t *)ptr);
  ptr += sizeof(uint32_t);

  namespace ser = ros::serialization;
  ser::IStream stream(ptr, msg_size);
  ser::deserialize(stream, msg);

  return msg_size + sizeof(MESSAGE_TYPE) + sizeof(uint32_t);
}

void odom_sub_udp_cb(const nav_msgs::OdometryPtr &msg)
{
  static ros::Time t_last;
  ros::Time t_now = ros::Time::now();
  if ((t_now - t_last).toSec() * odom_broadcast_freq_ < 1.0)
  {
    return;
  }
  t_last = t_now;

  msg->child_frame_id = string("ugv_") + std::to_string(self_id_);
  double t = msg->header.stamp.toSec();
  msg->header.stamp = ros::Time().fromSec(t - delta_time_);

  int len = serializeTopic(MESSAGE_TYPE::ODOM, *msg);

  if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0)
  {
    ROS_ERROR("UDP SEND ERROR (ODOM)!!!");
  }
}

void traj_sub_udp_cb(const traj_utils::BsplinePtr &msg)
{
  double t = msg->start_time.toSec();
  msg->start_time = ros::Time().fromSec(t - delta_time_);

  int len = serializeTopic(MESSAGE_TYPE::TRAJ, *msg);

  if (sendto(udp_send_fd_, udp_send_buf_, len, 0, (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0)
  {
    ROS_ERROR("UDP SEND ERROR (TRAJ)!!!");
  }
}

void udp_recv_fun()
{
  int valread;
  struct sockaddr_in addr_client;
  socklen_t addr_len;

  // Connect
  if (udp_bind_to_port(UDP_PORT, udp_server_fd_) < 0)
  {
    ROS_ERROR("[bridge_node]Socket recever creation error!");
    exit(EXIT_FAILURE);
  }

  while (true)
  {
    if ((valread = recvfrom(udp_server_fd_, udp_recv_buf_, BUF_LEN, 0, (struct sockaddr *)&addr_client, (socklen_t *)&addr_len)) < 0)
    {
      ROS_ERROR("recvfrom() < 0, error:");
      exit(EXIT_FAILURE);
    }

    char *ptr = udp_recv_buf_;
    switch (*((MESSAGE_TYPE *)ptr))
    {

    case MESSAGE_TYPE::ODOM:
    {
      nav_msgs::Odometry odom_msg_;
      if (valread == deserializeTopic(odom_msg_))
      {
        odom_msg_.header.stamp = ros::Time().fromSec(odom_msg_.header.stamp.toSec() + delta_time_);
        other_odom_pub_.publish(odom_msg_);
      }
      else
      {
        ROS_ERROR("Received message length not matches the sent one (ODOM)!!!");
        continue;
      }
      break;
    }
    case MESSAGE_TYPE::TRAJ:
    {
      traj_utils::Bspline Traj_msg_;
      if (valread == deserializeTopic(Traj_msg_))
      {
        Traj_msg_.start_time = ros::Time().fromSec(Traj_msg_.start_time.toSec() + delta_time_);
        if(Traj_msg_.drone_id != self_id_)
        {
          other_traj_pub_.publish(Traj_msg_);
        }
      }
      else
      {
        ROS_ERROR("Received message length not matches the sent one (TRAJ)!!!");
        continue;
      }
      break;
    }
    default:
      ROS_ERROR("Unknown received message type???");
      break;
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "swarm_bridge");
  ros::NodeHandle nh("~");

  nh.param("broadcast_ip", udp_ip_, string("127.0.0.1"));
  nh.param("self_id", self_id_, -1);
  nh.param("odom_max_freq", odom_broadcast_freq_, 10.0);

  if (self_id_ == -1)
  {
    ROS_WARN("[swarm bridge] Wrong self_id!");
    exit(EXIT_FAILURE);
  }

  my_odom_sub_ = nh.subscribe("broadcast_odom", 10, odom_sub_udp_cb, ros::TransportHints().tcpNoDelay());
  other_odom_pub_ = nh.advertise<nav_msgs::Odometry>("broadcast_odom2", 10);

  my_traj_sub_ = nh.subscribe("broadcast_bspline", 100, traj_sub_udp_cb, ros::TransportHints().tcpNoDelay());
  other_traj_pub_ = nh.advertise<traj_utils::Bspline>("broadcast_bspline2", 100);

  boost::thread udp_recv_thd(udp_recv_fun);
  udp_recv_thd.detach();
  ros::Duration(0.1).sleep();

  // UDP connect
  udp_send_fd_ = init_broadcast(udp_ip_.c_str(), UDP_PORT);

  cout << "[rosmsg_tcp_bridge] start running" << endl;

  ros::spin();

  close(udp_server_fd_);
  close(udp_send_fd_);

  return 0;
}