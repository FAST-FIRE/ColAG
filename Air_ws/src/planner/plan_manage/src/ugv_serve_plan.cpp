#include"plan_manage/ugv_serve_plan.h"


void ServePlan::init(ros::NodeHandle &nh)
{
  ugv_info_sub_ = nh.subscribe("/ellipse_collision_visualize", 1,  &ServePlan::UGVinfoCallback, this);
  waypoint_pub_ = nh.advertise<geometry_msgs::Pose>("/ellipse_collision_visualize", 1);
}

void ServePlan::UGVinfoCallback(const nav_msgs::OdometryConstPtr &msg)
{
  //store the collision time and pos
    std::string numstr = msg -> child_frame_id.substr(6);
    int id  = std::stoi(numstr);
    if (ugv_info_buf_.size() <= id)
    {
      for (size_t i = ugv_info_buf_.size(); i <= id; i++)
      {
        BlindUGVInfo blank;
        blank.robo_id = -1;
        ugv_info_buf_.push_back(blank);
      }
    }
    ugv_info_buf_[id].robo_id = id;
    ugv_info_buf_[id].colli_time = msg->header.stamp;
    ugv_info_buf_[id].colli_pos(0)= msg->pose.pose.position.y;
    //cout<<"here"<<endl;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "serve_plan");
  ros::NodeHandle nh("~");

  ServePlan rebo_replan;
  rebo_replan.init(nh);
  
  ros::spin();

  return 0;
}
