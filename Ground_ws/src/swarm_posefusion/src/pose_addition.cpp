#include <iostream>
#include <Eigen/Eigen>
// #include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <vector>
#include <map>
#include <math.h>
#include <tf/tf.h>
using namespace std;
using namespace Eigen;
int carnum, dronenum, self_id, master_id;
std::string frame;
VectorXd twist_input = VectorXd::Zero(6);
VectorXd pose_record = VectorXd::Zero(7);
Matrix<double, 7, 1> pose0 = VectorXd::Zero(7);
Matrix<double, 7, 1> pose3 = VectorXd::Zero(7);
bool have_self_odom = false;
ros::Time wheel_time;
class EKFinput
{
private:
  Matrix<double, 6, 6> Q;
  Matrix<double, 6, 6> R;
  Matrix<double, 6, 6> F;
  Matrix<double, 6, 6> H;

public:
  Matrix<double, 6, 1> preX;
  Matrix<double, 6, 1> nowX;
  Matrix<double, 6, 6> preP;
  Matrix<double, 6, 6> nowP;
  double safe_theta_ = M_PI / 4;
  int change_cout_[4] = {0};

  void filterInit(const VectorXd initX, double r, double q)
  {
    Q << 0.1, 0, 0, 0, 0, 0,
        0, 0.1, 0, 0, 0, 0,
        0, 0, 0.1, 0, 0, 0,
        0, 0, 0, 0.1, 0, 0,
        0, 0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0, 0.1;
    cout << "Q" << Q << endl;
    R << r, 0, 0, 0, 0, 0,
        0, r, 0, 0, 0, 0,
        0, 0, 10000.0, 0, 0, 0,
        0, 0, 0, 10000.0, 0, 0,
        0, 0, 0, 0, 10000.0, 0,
        0, 0, 0, 0, 0, r;
    // R.setIdentity(6,6);
    // R = R*r;

    cout << "R" << R << endl;
    F.setIdentity(6, 6);
    H.setIdentity(6, 6);
    nowX = initX;
    nowP.setIdentity(6, 6);
    nowP = nowP * 10000;
    cout << "initP" << nowP << endl;
  }
  void KalmanPredictor(const VectorXd input, double dt)
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

    // cout<<"now"<<nowX<<endl;
    for (int i = 3; i <= 5; i++)
    {
      while ((nowX(i)) > M_PI)
        nowX(i) -= 2 * M_PI;
      while ((nowX(i)) < -M_PI)
        nowX(i) += 2 * M_PI;
    }

    F(0, 5) = dt * (-input(0) * sin(preX(5)) - input(1) * cos(preX(5)));
    F(1, 5) = dt * (-input(1) * sin(preX(5)) + input(0) * cos(preX(5)));
    nowP = F * preP * F.transpose() + Q;
  }

  void KalmanUpdate(const VectorXd measure, double dt)
  {
    MatrixXd K = nowP * H.transpose() * (H * nowP * H.transpose() + R).inverse();
    VectorXd Z = H * nowX;
    VectorXd errorZ = (measure - Z);

    // 解决飘的很慢的情况 the measurement is slowly chenge
    Vector3d change1 = (nowX - preX).segment<3>(0);
    Vector3d change2 = (measure - preX).segment<3>(0);
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
        K.block<6, 6>(0, 0) = MatrixXd::Zero(6, 6);
      }
    }
    ////可以解决飘的很突然的情况 the measurement is suddenly chenge
    if (errorZ.segment<3>(0).norm() > 1.0)
    {
      ROS_ERROR("Position diff too much between measurement and prediction!!!");
      K.block<6, 6>(0, 0) = MatrixXd::Zero(6, 6);
    }
    else if (abs(errorZ(3)) > 3 || abs(errorZ(4)) > 3 || abs(errorZ(5)) > 3)
    {
      ROS_ERROR("Angle diff too much between measurement and prediction!!!");
      K.block<3, 6>(3, 0) = MatrixXd::Zero(3, 6);
    }

    nowX = nowX + K * errorZ;
    nowP = (MatrixXd::Identity(6, 6) - K * H) * nowP;
    // cout <<nowP<<endl;
  }

private:
  double acosIn2PI(Vector3d change1, Vector3d change2)
  {
    double theta;
    theta = acos(change1.dot(change2) / (change2.norm() * change2.norm()));
    if ((change1.cross(change2)).z() < 0)
    {
      theta = -theta;
    }
    return theta;
  }
};

class EKFdelta
{
private:
  Matrix<double, 6, 6> Q;
  Matrix<double, 6, 6> R;
  Matrix<double, 6, 6> F;
  Matrix<double, 6, 6> H;

public:
  Matrix<double, 6, 1> preX;
  Matrix<double, 6, 1> nowX;
  Matrix<double, 6, 6> preP;
  Matrix<double, 6, 6> nowP;

  void filterInit(const VectorXd initX, double r, double q)
  {
    Q << 0.1, 0, 0, 0, 0, 0,
        0, 0.1, 0, 0, 0, 0,
        0, 0, 0.1, 0, 0, 0,
        0, 0, 0, 0.1, 0, 0,
        0, 0, 0, 0, 0.1, 0,
        0, 0, 0, 0, 0, 0.1;

    R << r, 0, 0, 0, 0, 0,
        0, r, 0, 0, 0, 0,
        0, 0, 20.0, 0, 0, 0,
        0, 0, 0, 20.0, 0, 0,
        0, 0, 0, 0, 20.0, 0,
        0, 0, 0, 0, 0, r;

    F.setIdentity(6, 6);
    H.setIdentity(6, 6);
    nowX = initX;
    nowP << 10, 0, 0, 0, 0, 0,
        0, 10, 0, 0, 0, 0,
        0, 0, 10, 0, 0, 0,
        0, 0, 0, 10, 0, 0,
        0, 0, 0, 0, 10, 0,
        0, 0, 0, 0, 0, 10;
  }
  void KalmanPredictor(const VectorXd input)
  {
    preX = nowX;
    preP = nowP;

    // cout<<"input"<<input<<endl;
    nowX(0) = preX(0) + (input(0) * cos(preX(5)) - input(1) * sin(preX(5)));
    nowX(1) = preX(1) + (input(1) * cos(preX(5)) + input(0) * sin(preX(5)));
    nowX(2) = preX(2) + input(2);
    nowX(3) = preX(3) + input(3);
    nowX(4) = preX(4) + input(4);
    nowX(5) = preX(5) + input(5);

    // cout<<"now"<<nowX<<endl;
    for (int i = 3; i <= 5; i++)
    {
      while ((nowX(i)) > M_PI)
        nowX(i) -= 2 * M_PI;
      while ((nowX(i)) < -M_PI)
        nowX(i) += 2 * M_PI;
    }

    F(0, 5) = -input(0) * sin(preX(5)) - input(1) * cos(preX(5));
    F(1, 5) = -input(1) * sin(preX(5)) + input(0) * cos(preX(5));
    nowP = F * preP * F.transpose() + Q;
  }

  void KalmanUpdate(const VectorXd measure)
  {
    MatrixXd K = nowP * H.transpose() * (H * nowP * H.transpose() + R).inverse();
    VectorXd Z = H * nowX;

    nowX = nowX + K * (measure - Z);
    nowP = (MatrixXd::Identity(6, 6) - K * H) * nowP;
  }
};
class DataManager
{
private:
  int self_id;
  // max time tolarance between keyframe timestamp and message timestamp
  double MAX_TIME_TOLARANCE = 0.01;
  // min time between keyframes
  double MIN_TIME_BETWEEN_KEYFRAMES = 0.033;
  // time window in seconds
  double TIME_WINDOW_SECONDS = 2;

public:
  int robo_id;
  nav_msgs::Odometry relative_msg;
  std::vector<double> relative_msgs_new_timestamps;
  std::vector<double> relative_msgs_delete_timestamps;
  nav_msgs::Odometry odom_msg;
  std::vector<double> odom_msgs_new_timestamps;
  std::vector<double> odom_msgs_delete_timestamps;

  ros::Subscriber relative_sub;
  ros::Subscriber odom_sub;

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
  void rlswarmCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    relative_msg = *msg;
    have_rl = true;
  }

  void odomCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_msg = *msg;
    have_ms = true;
  }
  void clear_buffer()
  {
    relative_msgs_new_timestamps.clear();
    relative_msgs_delete_timestamps.clear();
    odom_msgs_new_timestamps.clear();
    odom_msgs_delete_timestamps.clear();
  }
};

void selfodomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  wheel_time = msg->header.stamp;
  twist_input << msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.linear.z,
      msg->twist.twist.angular.x,
      msg->twist.twist.angular.y,
      msg->twist.twist.angular.z;
  pose_record << msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w,
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z;
  have_self_odom = true;
}

int main(int argc, char **argv)
{

  // ROS_INFO("SWARM VO FUSE ROS\nIniting\n");

  // Use time as seed
  // srand(time(NULL));

  ros::init(argc, argv, "swarm_localization");
  ros::NodeHandle nh("~");

  int fusion_rate;
  double r, q;
  bool do_fusion;
  nh.param("fusion/car_num", carnum, 1);
  nh.param("fusion/drone_num", dronenum, 0);
  nh.param("fusion/self_id", self_id, 0);
  nh.param("fusion/master_id", master_id, 3);
  nh.param("fusion/frame", frame, string("world"));
  nh.param("fusion/rate", fusion_rate, 10);
  nh.param("fusion/doit", do_fusion, true);
  nh.param("fusion/r", r, 10.0);
  nh.param("fusion/q", q, 0.1);

  ros::Subscriber selfodom_sub = nh.subscribe("/odom", 10, selfodomCallback, ros::TransportHints().tcpNoDelay());

  // EKFinput  filter;
  EKFinput filter;
  bool filter_init = false;
  DataManager nuc[4];
  for (int i = 0; i < 4; i++)
  {
    if (i != self_id)
    {
      nuc[i].init(i, self_id);
      // string sub_topic_name = string("/nuc") + std::to_string(self_id) + string("/relative_pose2dev") + std::to_string(i) + string("_topic");
      //  string sub_topic_name = string("/result") + string("/robot") + std::to_string(i) + string("_pose");
      //  nuc[i].relative_sub = nh.subscribe(sub_topic_name, 10, &DataManager::rlswarmCallback, &nuc[i], ros::TransportHints().tcpNoDelay());
      //  sub_topic_name = string("/nuc") + std::to_string(i)  +  string("/Odometry");
      //  nuc[i].odom_sub = nh.subscribe(sub_topic_name, 10, &DataManager:: odomCallback,&nuc[i], ros::TransportHints().tcpNoDelay());

      string sub_topic_name = string("/result") + string("/robot") + std::to_string(i) + string("_pose");
      nuc[i].relative_sub = nh.subscribe(sub_topic_name, 10, &DataManager::rlswarmCallback, &nuc[i], ros::TransportHints().tcpNoDelay());
      sub_topic_name = string("/odom") + std::to_string(i) + string("_topic");
      nuc[i].odom_sub = nh.subscribe(sub_topic_name, 10, &DataManager::odomCallback, &nuc[i], ros::TransportHints().tcpNoDelay());
    }
  }
  ros::Publisher added_odom_pub = nh.advertise<nav_msgs::Odometry>("/added_odom", 10);
  ros::Publisher fused_odom_pub = nh.advertise<nav_msgs::Odometry>("/fused_odom", 10);
  ros::Time timePre;

  Matrix<double, 7, 1> pre_pose = VectorXd::Zero(7);
  bool relative_flag = true;
  int relative_cout = 0;
  ros::Rate rate(fusion_rate);
  while (nh.ok())
  {
    ros::spinOnce();
    if (do_fusion)
    {
      DataManager datamaster = nuc[master_id]; // 可以直接赋值吗
      if (datamaster.robo_id == master_id && datamaster.have_rl && datamaster.have_ms)
      {
        // if(abs((datamaster.relative_msg.header.stamp - datamaster.odom_msg.header.stamp).toSec() )> 1000.0)
        // {
        //     ROS_WARN("so long away!");
        // }
        // else
        {
          // double time1 = datamaster.relative_msg.header.stamp.toSec();
          // double time2 = datamaster.odom_msg.header.stamp.toSec();
          // cout<<time1<<"  "<<time2<<endl;
          // ROS_WARN("begin!");
          Eigen::Quaterniond relative_orientation, odom_orientation;
          Eigen::Vector3d relative_position, odom_position;
          ros::Time relative_time = datamaster.relative_msg.header.stamp;
          relative_orientation.x() = datamaster.relative_msg.pose.pose.orientation.x; // in my frame look master
          relative_orientation.y() = datamaster.relative_msg.pose.pose.orientation.y;
          relative_orientation.z() = datamaster.relative_msg.pose.pose.orientation.z;
          relative_orientation.w() = datamaster.relative_msg.pose.pose.orientation.w;
          relative_position.x() = datamaster.relative_msg.pose.pose.position.x;
          relative_position.y() = datamaster.relative_msg.pose.pose.position.y;
          relative_position.z() = datamaster.relative_msg.pose.pose.position.z;

          odom_orientation.x() = datamaster.odom_msg.pose.pose.orientation.x;
          odom_orientation.y() = datamaster.odom_msg.pose.pose.orientation.y;
          odom_orientation.z() = datamaster.odom_msg.pose.pose.orientation.z;
          odom_orientation.w() = datamaster.odom_msg.pose.pose.orientation.w;
          odom_position.x() = datamaster.odom_msg.pose.pose.position.x;
          odom_position.y() = datamaster.odom_msg.pose.pose.position.y;
          odom_position.z() = datamaster.odom_msg.pose.pose.position.z;

          // way1
          Eigen::Isometry3d T_ex1 = Eigen::Isometry3d::Identity();
          T_ex1.pretranslate(Eigen::Vector3d(-0.16, 0.0, 0.11));

          Eigen::Isometry3d T_1 = Eigen::Isometry3d::Identity();
          T_1.rotate(relative_orientation.normalized().toRotationMatrix());
          T_1.pretranslate(relative_position);
          T_1 = T_1.inverse();

          Eigen::Isometry3d T_2 = Eigen::Isometry3d::Identity();
          T_2.rotate(odom_orientation.normalized().toRotationMatrix());
          T_2.pretranslate(odom_position);

          Eigen::AngleAxisd v(M_PI / 2, Eigen::Vector3d::UnitZ());
          Eigen::Matrix3d rotMatrix = v.matrix();
          Eigen::Isometry3d T_ex2 = Eigen::Isometry3d::Identity();
          T_ex2.rotate(rotMatrix);
          T_ex2.pretranslate(Eigen::Vector3d(0, 0.02, 0.08));

          Eigen::Isometry3d T_3 = T_2 * T_ex2 * T_1 * T_ex1;
          Eigen::Vector3d final_position = T_3.translation();                        // t_odom_curr_now 平移向量
          Eigen::Quaterniond final_orientation = Eigen::Quaterniond(T_3.rotation()); // 转为四元数

          // way2
          //  Eigen::Vector3d final_position;
          //  Eigen::Quaterniond final_orientation;
          //  Eigen::Vector3d relative_position2 = relative_orientation.inverse()*(-relative_position)
          //  final_position = odom_orientation* (relative_position2) + odom_position;
          //  final_orientation = odom_orientation * relative_orientation.inverse();

          nav_msgs::Odometry odom_fused;
          odom_fused.header.frame_id = frame;
          odom_fused.child_frame_id = string("ugv_") + std::to_string(self_id);
          odom_fused.header.stamp = relative_time;
          odom_fused.pose.pose.position.x = final_position.x();
          odom_fused.pose.pose.position.y = final_position.y();
          odom_fused.pose.pose.position.z = final_position.z();
          odom_fused.pose.pose.orientation.x = final_orientation.x();
          odom_fused.pose.pose.orientation.y = final_orientation.y();
          odom_fused.pose.pose.orientation.z = final_orientation.z();
          odom_fused.pose.pose.orientation.w = final_orientation.w();

          added_odom_pub.publish(odom_fused);
          if (have_self_odom == true)
          {
            if (!filter_init)
            {
              VectorXd initx(6);
              // Eigen::Vector3d eulerAngle = final_orientation.matrix().eulerAngles(0,1,2);
              tf::Quaternion quat;
              tf::quaternionMsgToTF(odom_fused.pose.pose.orientation, quat);
              double roll, pitch, yaw;
              tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //
              initx << final_position, roll, pitch, yaw;

              filter.filterInit(initx, r, q);
              cout << "have init" << endl;
              timePre = ros::Time::now();
              filter_init = true;
              pre_pose << pose_record;
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

              //****way1*****//
              double dt = (ros::Time::now() - timePre).toSec();
              filter.KalmanPredictor(twist_input, dt);
              if (fabs((wheel_time - relative_time).toSec()) < 0.05)
              {
                if (!relative_flag)
                {
                  if (relative_cout < 8)
                    relative_cout++;
                  else
                  {
                    relative_flag = true;
                    relative_cout = 0;
                  }
                }
                else
                  // cout<<"update;"<<endl;
                  filter.KalmanUpdate(measure, dt);
              }
              else
              {
                if (relative_flag)
                  relative_flag = false;
                ROS_ERROR("longtime no relative!!!");
              }
              timePre = ros::Time::now();
              // Eigen::AngleAxisd rollAngle(AngleAxisd(filter.nowX(3),Vector3d::UnitX()));
              // Eigen::AngleAxisd pitchAngle(AngleAxisd(filter.nowX(4),Vector3d::UnitY()));
              // Eigen::AngleAxisd yawAngle(AngleAxisd(filter.nowX(5),Vector3d::UnitZ()));
              // Eigen::Quaterniond quaternion;
              // quaternion=rollAngle*pitchAngle*yawAngle;
              //****way1*****//

              //****way2*****//
              //     VectorXd now_pose = pose_record;
              //     Eigen::Quaterniond now_orient ( Vector4d(now_pose.block(3,0,4,1)));
              //     Eigen::Quaterniond pre_orient(Vector4d(pre_pose.block(3,0,4,1)));

              //     Eigen::Isometry3d Tnow = Eigen::Isometry3d::Identity();
              //     Tnow.rotate (now_orient.normalized().toRotationMatrix());
              //     Tnow.pretranslate(Vector3d(now_pose.block(0,0,3,1)));
              //     Eigen::Isometry3d Tpre= Eigen::Isometry3d::Identity();
              //     Tpre.rotate (pre_orient.normalized().toRotationMatrix());
              //     Tpre.pretranslate(Vector3d(pre_pose.block(0,0,3,1)));

              //     Eigen::Isometry3d Tdelta = Tpre.inverse()*Tnow;
              //     Eigen::Vector3d  delta_pos= Tdelta.translation();//t_odom_curr_now 平移向量
              //     Eigen::Quaterniond delta_orient = Eigen::Quaterniond (Tdelta.rotation());//转为四元数
              //     // Vector3d delta_pos = pre_orient.inverse()*(now_pose.block(0,0,3,1) - pre_pose.block(0,0,3,1));
              //     // Eigen::Quaterniond delta_orient = pre_orient.inverse()*now_orient;
              //    // Eigen::Vector3d delta_angle = delta_orient.matrix().eulerAngles(0,1,2);

              //     geometry_msgs::Quaternion qu;
              //     qu.x = delta_orient.x();
              //     qu.y = delta_orient.y();
              //     qu.z = delta_orient.z();
              //     qu.w = delta_orient.w();
              //     tf::quaternionMsgToTF(qu, quat);
              //     tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//

              //     VectorXd delta_pose(6);
              //     delta_pose << delta_pos,roll, pitch, yaw;
              //     filter.KalmanPredictor(delta_pose);
              //     filter.KalmanUpdate(measure);
              //     pre_pose = now_pose;
              //****way2*****//

              geometry_msgs::Quaternion q;
              q = tf::createQuaternionMsgFromRollPitchYaw(filter.nowX(3), filter.nowX(4), filter.nowX(5));
              nav_msgs::Odometry odom_filter;
              odom_filter.header.frame_id = frame;
              odom_filter.child_frame_id = string("ugv_") + std::to_string(self_id);
              odom_filter.header.stamp = wheel_time;
              odom_filter.pose.pose.position.x = filter.nowX(0);
              odom_filter.pose.pose.position.y = filter.nowX(1);
              odom_filter.pose.pose.position.z = filter.nowX(2);
              odom_filter.pose.pose.orientation = q;
              fused_odom_pub.publish(odom_filter);
            }
          }
          else
          {
            ROS_WARN("Waiting for selfodom");
          }
        }
      }
      else
      {
        ROS_WARN("NO ENOUGH INFO%d,%d,%d", datamaster.robo_id, datamaster.have_rl, datamaster.have_ms);
      }
    }
    rate.sleep();
  }
  ros::spin();

  return 0;
}

//**way3** //for delta//
// // 获取起始odom消息的位移与旋转
// tf::quaternionMsgToTF(start_odom_msg_.pose.pose.orientation, orientation);
// tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

// Eigen::Affine3f transBegin = pcl::getTransformation(
//     start_odom_msg_.pose.pose.position.x,
//     start_odom_msg_.pose.pose.position.y,
//     start_odom_msg_.pose.pose.position.z,
//     roll, pitch, yaw);

// // 获取终止odom消息的位移与旋转
// tf::quaternionMsgToTF(end_odom_msg_.pose.pose.orientation, orientation);
// tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

// Eigen::Affine3f transEnd = pcl::getTransformation(
//     end_odom_msg_.pose.pose.position.x,
//     end_odom_msg_.pose.pose.position.y,
//     end_odom_msg_.pose.pose.position.z,
//     roll, pitch, yaw);

// // 求得这之间的变换
// Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

// // 通过　transBt　获取　odomIncreX等，一帧点云数据时间的odom变化量
// float rollIncre, pitchIncre, yawIncre;
// pcl::getTranslationAndEulerAngles(transBt,
//                                   odom_incre_x_, odom_incre_y_, odom_incre_z_,
//                                   rollIncre, pitchIncre, yawIncre);
//**way3***//