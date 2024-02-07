#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen/Eigen>

Eigen::Vector3d p0_init, p1_init;
Eigen::Quaterniond q0_init, q1_init;

Eigen::Vector3d true_relative_position;
Eigen::Quaterniond true_relative_orientation;
// double true_pose_update_time;
ros::Publisher gt_pose_pub, gt_path_pub, real_pose_pub;

ros::Subscriber model_states_sub;
double last_sub_time;
std::string other_name, my_name;

void Callback(const nav_msgs::OdometryConstPtr true_pose0, const nav_msgs::OdometryConstPtr true_pose1)
{
    std::cout<<"in it"<<std::endl;
    Eigen::Vector3d position0, position1;
    position0 << true_pose0->pose.pose.position.x, true_pose0->pose.pose.position.y, true_pose0->pose.pose.position.z;
    position1 << true_pose1->pose.pose.position.x, true_pose1->pose.pose.position.y, true_pose1->pose.pose.position.z;
    Eigen::Quaterniond orien0(true_pose0->pose.pose.orientation.w, true_pose0->pose.pose.orientation.x, true_pose0->pose.pose.orientation.y, true_pose0->pose.pose.orientation.z);
    Eigen::Quaterniond orien1(true_pose1->pose.pose.orientation.w, true_pose1->pose.pose.orientation.x, true_pose1->pose.pose.orientation.y, true_pose1->pose.pose.orientation.z);
    true_relative_position = orien0.conjugate() * (orien1 * p1_init + position1 - (orien0 * p0_init + position0));
    true_relative_orientation = orien0.conjugate() * orien1;
    // true_pose_update_time = ros::Time::now().toSec();

    // publish odometry and path data
    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id = "world";
    odom_msg.header.stamp = true_pose0->header.stamp;
    odom_msg.pose.pose.position.x = true_relative_position(0);
    odom_msg.pose.pose.position.y = true_relative_position(1);
    odom_msg.pose.pose.position.z = true_relative_position(2);
    odom_msg.pose.pose.orientation.w = true_relative_orientation.w();
    odom_msg.pose.pose.orientation.x = true_relative_orientation.x();
    odom_msg.pose.pose.orientation.y = true_relative_orientation.y();
    odom_msg.pose.pose.orientation.z = true_relative_orientation.z();
    gt_pose_pub.publish(odom_msg);
}

void robotStatesCallback(const gazebo_msgs::ModelStates &currents_states)
{
    ros::Time now_time = ros::Time::now();
    if ((now_time.toSec() - last_sub_time) <= 0.01)
        return;
    int other_index = -1;
    int my_index = -1;
    std::vector<std::string> model_names = currents_states.name;
    for (size_t i = 0; i < model_names.size(); i++)
    {
        if (model_names[i] == other_name)
            other_index = i;
        if (model_names[i] == my_name)
            my_index = i;
    }
    if (other_index < 0 || my_index < 0)
    {
        ROS_ERROR("NO NEEDED NAME,%s,%s", other_name.c_str(), my_name.c_str());
        return;
    }
    geometry_msgs::Pose true_pose0, true_pose1;
    true_pose0 = currents_states.pose[my_index];
    true_pose1 = currents_states.pose[other_index];
    Eigen::Vector3d position0, position1;
    position0 << true_pose0.position.x, true_pose0.position.y, true_pose0.position.z;
    position1 << true_pose1.position.x, true_pose1.position.y, true_pose1.position.z;
    Eigen::Quaterniond orien0(true_pose0.orientation.w, true_pose0.orientation.x, true_pose0.orientation.y, true_pose0.orientation.z);
    Eigen::Quaterniond orien1(true_pose1.orientation.w, true_pose1.orientation.x, true_pose1.orientation.y, true_pose1.orientation.z);
    true_relative_position = orien0.conjugate() * (orien1 * p1_init + position1 - (orien0 * p0_init + position0));
    true_relative_orientation = orien0.conjugate() * orien1;
    // true_pose_update_time = ros::Time::now().toSec();

    // publish odometry and path data
    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id = "world";
    odom_msg.header.stamp = now_time;
    odom_msg.pose.pose.position.x = true_relative_position(0);
    odom_msg.pose.pose.position.y = true_relative_position(1);
    odom_msg.pose.pose.position.z = true_relative_position(2);
    odom_msg.pose.pose.orientation.w = true_relative_orientation.w();
    odom_msg.pose.pose.orientation.x = true_relative_orientation.x();
    odom_msg.pose.pose.orientation.y = true_relative_orientation.y();
    odom_msg.pose.pose.orientation.z = true_relative_orientation.z();
    gt_pose_pub.publish(odom_msg);

    nav_msgs::Odometry selfodom_msg;
    selfodom_msg.header.frame_id = "world";
    selfodom_msg.header.stamp = now_time;
    selfodom_msg.pose.pose.position.x = position0(0);
    selfodom_msg.pose.pose.position.y = position0(1);
    selfodom_msg.pose.pose.position.z = position0(2);
    selfodom_msg.pose.pose.orientation.w = orien0.w();
    selfodom_msg.pose.pose.orientation.x = orien0.x();
    selfodom_msg.pose.pose.orientation.y = orien0.y();
    selfodom_msg.pose.pose.orientation.z = orien0.z();
    real_pose_pub.publish(selfodom_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_error");
    ros::NodeHandle nh("~");

    ROS_INFO("hello");
    std::vector<double> p0_ex, p1_ex;
    bool gazebo_sim;

    nh.param<std::string>("other_name_", other_name, "other_name");
    nh.param<std::string>("my_name_", my_name, "my_name");
    nh.param("gazeboll_sim_", gazebo_sim, true);
    nh.getParam("p0_ex_", p0_ex);
    nh.getParam("p1_ex_", p1_ex);
    p0_init = Eigen::Vector3d(p0_ex[0], p0_ex[1], p0_ex[2]);
    p1_init = Eigen::Vector3d(p1_ex[0], p1_ex[1], p1_ex[2]);

    // p0_init = q0_init * (p0m - pc0) + pc0_uav -  puav0;
    // p1_init = q1_init * (p1m - pc1) + pc1_uav - puav1 ;
    // p0_init << Eigen::Vector3d(-5.625, 0.193512, 0.36898) -  Eigen::Vector3d(-5.72308984375, 0.19648825073242188, 0.28556573486328124);
    // p1_init << Eigen::Vector3d(-5.61013, 0.1961122, 0.281028) - Eigen::Vector3d(-5.72358935546875, 0.1890719451904297, 0.20312228393554688);

    // lidar (the result is based p0)
    // p0_init << Eigen::Vector3d(-6.12544749, 2.63567006, 0.921985110) -  Eigen::Vector3d(-6.276048635, 2.620090482, 0.740647820);
    // RM2
    // p1_init << Eigen::Vector3d(-6.36430699, 2.46240500, 0.396670000) - Eigen::Vector3d(-6.498386951, 2.482511077, 0.240478539);
    if (gazebo_sim)
    {
        model_states_sub = nh.subscribe("/gazebo/model_states", 1, robotStatesCallback, ros::TransportHints().tcpNoDelay());
    }
    else
    {
        message_filters::Subscriber<nav_msgs::Odometry> pose0_ground_truth_sub(nh, "/drone_2_visual_slam/odom", 1, ros::TransportHints().tcpNoDelay());
        message_filters::Subscriber<nav_msgs::Odometry> pose1_ground_truth_sub(nh, "/drone_99_visual_slam/odom", 1, ros::TransportHints().tcpNoDelay());
        typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> SyncPolicy;
        message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(5), pose0_ground_truth_sub, pose1_ground_truth_sub);
        sync.registerCallback(boost::bind(&Callback, _1, _2));
    }
    gt_pose_pub = nh.advertise<nav_msgs::Odometry>("/true/robot0_pose", 1);
    real_pose_pub = nh.advertise<nav_msgs::Odometry>("/true/odom", 1);

    ros::spin();
    return 0;
}
