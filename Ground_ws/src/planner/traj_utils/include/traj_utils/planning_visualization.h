#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <algorithm>
//#include <bspline_opt/uniform_bspline.h>
#include <iostream>
//#include <bspline_opt/polynomial_traj.h>
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <stdlib.h>

using std::vector;
namespace ego_planner
{
  class PlanningVisualization
  {
  private:
    ros::NodeHandle node;

    ros::Publisher goal_point_pub;
    ros::Publisher global_list_pub;
    ros::Publisher init_list_pub;
    ros::Publisher optimal_list_pub;
    ros::Publisher a_star_list_pub;
    ros::Publisher guide_vector_pub;
    ros::Publisher intermediate_state_pub;
    ros::Publisher visible_region_pub;
    

    enum Color { white,
             red,
             green,
             blue,
             yellow,
             greenblue };

  public:
    PlanningVisualization(/* args */) {}
    ~PlanningVisualization() {}
    PlanningVisualization(ros::NodeHandle &nh);

    typedef std::shared_ptr<PlanningVisualization> Ptr;

    void displayMarkerList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale,
                           Eigen::Vector4d color, int id,  bool show_sphere = true);
    void generatePathDisplayArray(visualization_msgs::MarkerArray &array,
                                  const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void generateArrowDisplayArray(visualization_msgs::MarkerArray &array,
                                   const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id);
    void displayGlobalPathList(vector<Eigen::Vector3d> global_pts, const double scale, int id);
    void displayInitPathList(vector<Eigen::Vector3d> init_pts, const double scale, int id);
    void displayMultiInitPathList(vector<vector<Eigen::Vector3d>> init_trajs, const double scale);
    void displayOptimalList(Eigen::MatrixXd optimal_pts, int id);
    void displayAStarList(std::vector<std::vector<Eigen::Vector3d>> a_star_paths, int id);
    void displayArrowList(ros::Publisher &pub, const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);

    void visualize_fan_shape_meshes(const std::vector<Eigen::Vector3d>& v0, const std::vector<Eigen::Vector3d>& v1, 
                                                                            const std::vector<double>& thetas);
    // void displayIntermediateState(ros::Publisher& intermediate_pub, ego_planner::BsplineOptimizer::Ptr optimizer, double sleep_time, const int start_iteration);
    // void displayNewArrow(ros::Publisher& guide_vector_pub, ego_planner::BsplineOptimizer::Ptr optimizer);


    void setMarkerPose(visualization_msgs::Marker& marker,
                     const double& x,
                     const double& y,
                     const double& z) 
  {
      marker.pose.position.x = x;
      marker.pose.position.y = y;
      marker.pose.position.z = z;
      marker.pose.orientation.w = 1;
      marker.pose.orientation.x = 0;
      marker.pose.orientation.y = 0;
      marker.pose.orientation.z = 0;
  }

  void setMarkerScale(visualization_msgs::Marker& marker,
                      const double& x,
                      const double& y,
                      const double& z) 
  {
    marker.scale.x = x;
    marker.scale.y = y;
    marker.scale.z = z;
  }

  void setMarkerColor(visualization_msgs::Marker& marker,
                        Color color = Color::blue,
                        double a = 1) 
    {
      marker.color.a = a;
      switch (color) 
      {
        case white:
          marker.color.r = 1;
          marker.color.g = 1;
          marker.color.b = 1;
          break;
        case red:
          marker.color.r = 1;
          marker.color.g = 0;
          marker.color.b = 0;
          break;
        case green:
          marker.color.r = 0;
          marker.color.g = 1;
          marker.color.b = 0;
          break;
        case blue:
          marker.color.r = 0;
          marker.color.g = 0;
          marker.color.b = 1;
          break;
        case yellow:
          marker.color.r = 1;
          marker.color.g = 1;
          marker.color.b = 0;
          break;
        case greenblue:
          marker.color.r = 0;
          marker.color.g = 1;
          marker.color.b = 1;
          break;
      }
    }
  };
} // namespace ego_planner
#endif