#include "CollisionDetect.h"

void CollisionDetect::init(ros::NodeHandle &nh)
{
  double q, r;
  nh.param("collision/q", q, 0.01);
  nh.param("collision/r", r, 1.0);
  nh.param("collision/self_id", self_id, 0);
  nh.param("collision/det_range", det_range_, 3.0);
  std::vector<double> pos;
  nh.getParam("collision/bound_max", pos);
  bound_max_ << pos[0],pos[1], pos[2];
  nh.getParam("collision/bound_min", pos);
  bound_min_ << pos[0],pos[1], pos[2];

  ekf.reset(new PoseEKF(q, r));
  ellipse_pub = nh.advertise<visualization_msgs::MarkerArray>("ellipse_collision_visualize", 1);
  collision_point_pub = nh.advertise<visualization_msgs::Marker>("collision_point_visualize", 1);
  blind_info_pub = nh.advertise<custom_msgs::blind_info>("blind_info", 1);

  last_collision_info.first = -1;
  last_collision_info.second(0) = 0;
  last_collision_info.second(1) = 0;
  last_collision_info.second(2) = 0;
}


double CollisionDetect::caculate_collision(GridMap::Ptr grid_map,
                                           ego_planner::UniformBspline bspline,
                                           ros::Time bspline_start_time,
                                           Eigen::Vector3d odom_pos,
                                           Eigen::Vector3d odom_vel,
                                           Eigen::Quaterniond odom_orient,
                                           Eigen::Matrix<double, 6, 6> odom_cov)
{
  // grid_map: 地图
  // bspline: 轨迹
  // odom_pos: 里程计位置
  // odom_vel: 里程计速度
  // odom_orient: 里程计姿态
  // odom_cov: 里程计协方差

  marker_array.markers.clear();

  double dt = 0.2;
  int id = 0;

  ros::Time now_time = ros::Time::now();
  double ekf_start_time = (now_time - bspline_start_time).toSec();

  // // 重置EKF初始状态为当前里程计
  // Eigen::VectorXd initX;
  // initX = Eigen::VectorXd::Zero(6);
  // // 将odom_orient转换为欧拉角
  // tf::Quaternion q(odom_orient.x(), odom_orient.y(), odom_orient.z(), odom_orient.w());
  // double roll, pitch, yaw;
  // tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

  // // initX(0) = odom_pos(0);
  // // initX(1) = odom_pos(1);
  // // initX(2) = odom_pos(2);
  // // initX(3) = 0;
  // // initX(4) = 0;
  // // initX(5) = yaw;

  // 重置EKF初始状态为当前轨迹起点
  Eigen::VectorXd initX;
  initX = Eigen::VectorXd::Zero(6);

  auto pos0 = bspline.evaluateDeBoorT(ekf_start_time);
  auto vel0 = bspline.getDerivative().evaluateDeBoorT(ekf_start_time);

  initX(0) = pos0(0);
  initX(1) = pos0(1);
  initX(2) = pos0(2);
  initX(3) = 0;
  initX(4) = 0;
  initX(5) = atan2(vel0(1), vel0(0));

  ekf->filterInit(initX);
  ekf->nowP = odom_cov;

  // 按照时间步长遍历轨迹
  for (double time_i = ekf_start_time; time_i < bspline.getTimeSum(); time_i += dt)
  {
    // 计算轨迹在当前时间步长的位置、速度
    Eigen::Vector3d pos, vel, acc;
    pos = bspline.evaluateDeBoorT(time_i);
    vel = bspline.getDerivative().evaluateDeBoorT(time_i);
    acc = bspline.getDerivative().getDerivative().evaluateDeBoorT(time_i);
    
    if((pos0 - pos).norm() > det_range_)
    {
      break;
    }

    // 计算线速度
    double v;
    v = vel.norm();
    // 计算角速度
    double w;
    w = vel.cross(acc)(2) / vel.squaredNorm();

    // 更新EKF
    Eigen::Matrix<double, 6, 1> input = Eigen::Matrix<double, 6, 1>::Zero();
    input(0) = v;
    input(5) = w;
    ekf->kalmanPredict(input, dt);

    Eigen::Matrix2d cor_cov_mat;  
    cor_cov_mat = ekf->nowP.block(0, 0, 2, 2);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cor_cov_mat);
    Eigen::Vector2d axisvalues = es.eigenvalues();
    Eigen::Matrix2d eigenvectors = es.eigenvectors();
    // std::cout << "ellipse values1 " << axisvalues.transpose()<< std::endl;
    axisvalues(0) = sqrt(axisvalues(0))*3;//- 0.300;
    axisvalues(1) = sqrt(axisvalues(1))*3;//- 0.300;
    // std::cout << "ellipse values2 " << axisvalues.transpose()<< std::endl;
    if(axisvalues(0) < 0) axisvalues(0) = 0;
    if(axisvalues(1) < 0) axisvalues(1) = 0;

    marker_array.markers.push_back(ellipse_marker(pos, axisvalues, eigenvectors, id));
    std::pair<bool, Eigen::Vector3d> collision_info;

    collision_info = ellipse_check(grid_map, pos, axisvalues, eigenvectors);
    // 检测碰撞
    if (collision_info.first)
    {
      // 发生碰撞
      marker_array.markers.back().color.r = 1.0;
      marker_array.markers.back().color.g = 0.0;
      marker_array.markers.back().color.b = 0.0;
      marker_array.markers.back().color.a = 0.5;

      double to_colli_time = time_i - ekf_start_time;
      double global_colli_time = now_time.toSec() + to_colli_time;
      std::cout << "after how much time collide: " << to_colli_time << std::endl;

      if (((collision_info.second - last_collision_info.second).norm() > 0.7 && fabs(global_colli_time - last_collision_info.first) > 0.4) || fabs(global_colli_time - last_collision_info.first) > 3.0 || last_collision_info.first == -1)
      {
        custom_msgs::blind_info blind_info;
        blind_info.header.stamp = now_time;
        blind_info.header.frame_id = "map";
        blind_info.id = self_id;
        blind_info.collision_time = to_colli_time;
        
        Eigen::Vector3d vis_point;
        vis_point = choosePoint(collision_info.second, vel);
        blind_info.collision_x = vis_point(0);
        blind_info.collision_y = vis_point(1);
        blind_info.collision_z = vis_point(2);
        blind_info_pub.publish(blind_info);

        last_collision_info.first = global_colli_time;
        last_collision_info.second = collision_info.second;
      }

      return to_colli_time;
    }
    id++;
  }
  // 未发生碰撞
  if (last_collision_info.first != -1)
  {
    custom_msgs::blind_info blind_info;
    blind_info.header.stamp = ros::Time::now();
    blind_info.header.frame_id = "map";
    blind_info.id = self_id;
    blind_info.collision_time = -1;
    blind_info.collision_x = 0;
    blind_info.collision_y = 0;
    blind_info.collision_z = 0;
    blind_info_pub.publish(blind_info);

    last_collision_info.first = -1;
    last_collision_info.second = Eigen::Vector3d(0, 0, 0);
  }

  double end_time = ros::Time::now().toSec();
  // std::cout << "compute time: " << end_time - start_time << std::endl;
  // std::cout << "collision time: nan" << std::endl;
  return -1;
}

void CollisionDetect::visualize_collision()
{
  while (marker_array.markers.size() > 100)
  {
    marker_array.markers.erase(marker_array.markers.begin() + 100);
  }
  for (int i = marker_array.markers.size(); i < 100; i++)
  {
    marker_array.markers.push_back(ellipse_marker(Eigen::Vector3d(0, 0, 0), Eigen::Matrix<double, 2, 2>::Zero(), i));
  }

  ellipse_pub.publish(marker_array);
}

std::pair<bool, Eigen::Vector3d> CollisionDetect::ellipse_check(GridMap::Ptr grid_map,
                                                                Eigen::Vector3d odom_pos,
                                                                Eigen::Vector2d ellipse_axis,
                                                                Eigen::Matrix2d eigen_vectors)
{
  // odom_cov is the covariance matrix of the ellipse
  // odom_pos is the center of the ellipse
  // detect collision between ellipse and grid map

  // 从协方差矩阵获取长轴和短轴

  double a = ellipse_axis(0);
  double b = ellipse_axis(1);
  Eigen::Vector2d v1 = eigen_vectors.col(0);
  Eigen::Vector2d v2 = eigen_vectors.col(1);
  // std::cout << "a: " << a << " b: " << b << std::endl;
  if(a==0 ||b==0)
  {
    Eigen::Vector3d p_in_map;
    p_in_map(0) = odom_pos(0);
    p_in_map(1) = odom_pos(1);
    p_in_map(2) = 0.3;
              // 点在椭圆内，检测是否在障碍物内
    if (grid_map->getUnknownInflateOccupancy(p_in_map))
    {
      std::pair<bool, Eigen::Vector3d> info;
      // 点在障碍物内
      info.first = true;
      // info.second = p_in_map;//
      info.second = odom_pos;
      visualization_msgs::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = ros::Time();
      marker.ns = "collision_point_visualize";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = p_in_map(0);
      marker.pose.position.y = p_in_map(1);
      marker.pose.position.z = 0.3;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.a = 1;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      collision_point_pub.publish(marker);
      return info;
    }
  }
  else
  {
    double l = max(a, b);
    // 遍历椭圆占用的栅格
    for (double i = -l; i <= l; i += grid_map->getResolution())
    {
      for (double j = -l; j <= l; j += grid_map->getResolution())
      {
        // 计算点是否在椭圆内
        Eigen::Vector2d p = Eigen::Vector2d(i, j);
        if (p.dot(v1) * (p.dot(v1)) / a / a + (p.dot(v2)) * (p.dot(v2)) / b / b < 1)
        {
          Eigen::Vector3d p_in_map;
          p_in_map(0) = p(0) + odom_pos(0);
          p_in_map(1) = p(1) + odom_pos(1);
          p_in_map(2) = 0.3;

          // std::cout << "p_in_map: " << p_in_map << std::endl;
          // std::cout << "occupancy: " << grid_map->getInflateOccupancy(p_in_map) << std::endl;

          // 点在椭圆内，检测是否在障碍物内
          if (grid_map->getUnknownInflateOccupancy(p_in_map))
          {
            std::pair<bool, Eigen::Vector3d> info;
            // 点在障碍物内
            info.first = true;
            // info.second = p_in_map;//
            info.second = odom_pos;
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time();
            marker.ns = "collision_point_visualize";
            marker.id = 0;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = p_in_map(0);
            marker.pose.position.y = p_in_map(1);
            marker.pose.position.z = 0.3;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 1;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
            collision_point_pub.publish(marker);
            return info;
          }
        }
      }
    }
  }
 
  std::pair<bool, Eigen::Vector3d> info;
  info.first = false;
  return info;
}

std::pair<bool, Eigen::Vector3d> CollisionDetect::ellipse_check(GridMap::Ptr grid_map,
                                                                Eigen::Vector3d odom_pos,
                                                                Eigen::Matrix<double, 2, 2> odom_cov)
{
  // odom_cov is the covariance matrix of the ellipse
  // odom_pos is the center of the ellipse
  // detect collision between ellipse and grid map

  // std::cout << "odom_cov: " << std::endl;
  // std::cout << odom_cov << std::endl;
  // std::cout << std::endl;

  // 从协方差矩阵获取长轴和短轴
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(odom_cov);
  Eigen::Vector2d eigenvalues = es.eigenvalues();
  Eigen::Matrix2d eigenvectors = es.eigenvectors();
  double a = sqrt(eigenvalues(0))*3;
  double b = sqrt(eigenvalues(1))*3;
  cout<<"cov     "<<odom_cov<<endl;
  cout<<"ellipse "<<a<<" "<<b<<endl;
  Eigen::Vector2d v1 = eigenvectors.col(0);
  Eigen::Vector2d v2 = eigenvectors.col(1);
  // std::cout << "a: " << a << " b: " << b << std::endl;

  double l = max(a, b);
  // 遍历椭圆占用的栅格
  for (double i = -l; i <= l; i += grid_map->getResolution())
  {
    for (double j = -l; j <= l; j += grid_map->getResolution())
    {
      // 计算点是否在椭圆内
      Eigen::Vector2d p = Eigen::Vector2d(i, j);
      if (p.dot(v1) * (p.dot(v1)) / a / a + (p.dot(v2)) * (p.dot(v2)) / b / b < 1)
      {
        Eigen::Vector3d p_in_map;
        p_in_map(0) = p(0) + odom_pos(0);
        p_in_map(1) = p(1) + odom_pos(1);
        p_in_map(2) = 0.3;

        // std::cout << "p_in_map: " << p_in_map << std::endl;
        // std::cout << "occupancy: " << grid_map->getInflateOccupancy(p_in_map) << std::endl;

        // 点在椭圆内，检测是否在障碍物内
        if (grid_map->getUnknownInflateOccupancy(p_in_map))
        {
          std::pair<bool, Eigen::Vector3d> info;
          // 点在障碍物内
          info.first = true;
          // info.second = p_in_map;//
          info.second = odom_pos;
          visualization_msgs::Marker marker;
          marker.header.frame_id = "world";
          marker.header.stamp = ros::Time();
          marker.ns = "collision_point_visualize";
          marker.id = 0;
          marker.type = visualization_msgs::Marker::SPHERE;
          marker.action = visualization_msgs::Marker::ADD;
          marker.pose.position.x = p_in_map(0);
          marker.pose.position.y = p_in_map(1);
          marker.pose.position.z = 0.3;
          marker.scale.x = 0.1;
          marker.scale.y = 0.1;
          marker.scale.z = 0.1;
          marker.color.a = 1;
          marker.color.r = 0.0;
          marker.color.g = 0.0;
          marker.color.b = 1.0;
          collision_point_pub.publish(marker);
          return info;
        }
      }
    }
  }
  std::pair<bool, Eigen::Vector3d> info;
  info.first = false;
  return info;
}

visualization_msgs::Marker CollisionDetect::ellipse_marker(Eigen::Vector3d odom_pos,
                                                           Eigen::Vector2d ellipse_axis,
                                                           Eigen::Matrix2d eigen_vectors,
                                                           int id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = "collision_visualize";
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = odom_pos(0);
  marker.pose.position.y = odom_pos(1);
  marker.pose.position.z = 0.3;

  double a = ellipse_axis(0)*2;
  double b = ellipse_axis(1)*2;
  Eigen::Vector2d v1 = eigen_vectors.col(0);
  Eigen::Vector2d v2 = eigen_vectors.col(1);

  // 计算旋转角度
  double yaw = atan2(v1(1), v1(0));
  Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
  marker.pose.orientation.w = q.w();

  // 设置椭圆大小
  marker.scale.x = a;
  marker.scale.y = b;
  marker.scale.z = 0.1;
  marker.color.a = 0.05;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  return marker;
}

visualization_msgs::Marker CollisionDetect::ellipse_marker(Eigen::Vector3d odom_pos,
                                                           Eigen::Matrix<double, 2, 2> odom_cov,
                                                           int id)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = "collision_visualize";
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = odom_pos(0);
  marker.pose.position.y = odom_pos(1);
  marker.pose.position.z = 0.3;

  // 从协方差矩阵获取长轴和短轴
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(odom_cov);
  Eigen::Vector2d eigenvalues = es.eigenvalues();
  Eigen::Matrix2d eigenvectors = es.eigenvectors();
  double a = sqrt(eigenvalues(0))*3*2;
  double b = sqrt(eigenvalues(1))*3*2;
  Eigen::Vector2d v1 = eigenvectors.col(0);
  Eigen::Vector2d v2 = eigenvectors.col(1);

  // 计算旋转角度
  double yaw = atan2(v1(1), v1(0));
  Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
  marker.pose.orientation.w = q.w();

  // 设置椭圆大小
  marker.scale.x = a;
  marker.scale.y = b;
  marker.scale.z = 0.1;
  marker.color.a = 0.05;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  return marker;
}

Eigen::Vector3d CollisionDetect::choosePoint(Eigen::Vector3d pos, Eigen::Vector3d vel)
{
  Eigen::Vector3d get_point;
  Eigen::Vector3d dir = vel;
  dir(2) = 0;
  dir = dir.normalized();

  get_point = pos - 2.0 * dir;
  if(isInBound(get_point) > 0)
  {
    int cho = isInBound(pos);
    if(cho > 0)
    {
      get_point = pos + 2.0 * dir;
      if(isInBound(get_point) > 0)
      {
        if (pos(0) < bound_min_(0) + 1e-4)
        {
          get_point(0) = bound_min_(0);
        }
        else if (pos(0) > bound_max_(0) - 1e-4)
        {
          get_point(0) = bound_max_(0);
        }
        if (pos(1) < bound_min_(1) + 1e-4)
        {
          get_point(1) = bound_min_(1);
        }
        else if (pos(1) > bound_max_(1) - 1e-4)
        {
          get_point(1) = bound_max_(1);
        }
      }
    }
    else
    {
      // cout<<"no_bound_point "<<get_point.transpose()<<"pos"<<pos.transpose()<<endl;
      get_point = closetPointInBound(get_point, pos);
    }
  }
  return get_point;
}

Eigen::Vector3d CollisionDetect::closetPointInBound(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)
{
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = bound_max_ - camera_pt;
  Eigen::Vector3d min_tc = bound_min_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 2; ++i)
  {
    if (fabs(diff[i]) > 0)
    {
      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }
  cout<<"min_t"<<min_t<<endl;
  return camera_pt + (min_t - 1e-3) * diff;
}

int CollisionDetect::isInBound(const Eigen::Vector3d &pos)
{
  if (pos(0) < bound_min_(0) + 1e-4)
  {
    // cout << "less than min range!" << endl;
    return 1;
  }
  else if (pos(1) < bound_min_(1) + 1e-4)
  {
    return 2;
  }
  else if (pos(0) > bound_max_(0) - 1e-4)
  {
    return 3;
  }
  else if (pos(1) > bound_max_(1) - 1e-4)
  {
    return 4;
  }
  return 0;
}

