#include "plan_env/grid_map.h"

// #define current_img_ md_.depth_image_[image_cnt_ & 1]
// #define last_img_ md_.depth_image_[!(image_cnt_ & 1)]

void GridMap::initMap(ros::NodeHandle &nh)
{
  node_ = nh;

  /* get parameter */
  double x_size, y_size, z_size;
  node_.param("grid_map/resolution", mp_.resolution_, -1.0);
  node_.param("grid_map/map_size_x", x_size, -1.0);
  node_.param("grid_map/map_size_y", y_size, -1.0);
  node_.param("grid_map/map_size_z", z_size, -1.0);
  node_.param("grid_map/local_update_range_x", mp_.local_update_range_(0), -1.0);
  node_.param("grid_map/local_update_range_y", mp_.local_update_range_(1), -1.0);
  node_.param("grid_map/local_update_range_z", mp_.local_update_range_(2), -1.0);
  node_.param("grid_map/obstacles_inflation", mp_.obstacles_inflation_, -1.0);
  node_.param("grid_map/obstacles_inflation_z", mp_.obstacles_inflation_z_, -1.0);

  node_.param("grid_map/fx", mp_.fx_, -1.0);
  node_.param("grid_map/fy", mp_.fy_, -1.0);
  node_.param("grid_map/cx", mp_.cx_, -1.0);
  node_.param("grid_map/cy", mp_.cy_, -1.0);

  node_.param("grid_map/use_depth_filter", mp_.use_depth_filter_, true);
  node_.param("grid_map/depth_filter_tolerance", mp_.depth_filter_tolerance_, -1.0);
  node_.param("grid_map/depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);
  node_.param("grid_map/depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);
  node_.param("grid_map/depth_filter_margin", mp_.depth_filter_margin_, -1);
  node_.param("grid_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);
  node_.param("grid_map/skip_pixel", mp_.skip_pixel_, -1);

  node_.param("grid_map/p_hit", mp_.p_hit_, 0.70);
  node_.param("grid_map/p_miss", mp_.p_miss_, 0.35);
  node_.param("grid_map/p_min", mp_.p_min_, 0.12);
  node_.param("grid_map/p_max", mp_.p_max_, 0.97);
  node_.param("grid_map/p_occ", mp_.p_occ_, 0.80);
  node_.param("grid_map/min_ray_length", mp_.min_ray_length_, -0.1);
  node_.param("grid_map/max_ray_length", mp_.max_ray_length_, -0.1);

  node_.param("grid_map/visualization_truncate_height", mp_.visualization_truncate_height_, -0.1);
  node_.param("grid_map/virtual_ceil_height", mp_.virtual_ceil_height_, -0.1);
  node_.param("grid_map/virtual_ceil_yp", mp_.virtual_ceil_yp_, -0.1);
  node_.param("grid_map/virtual_ceil_yn", mp_.virtual_ceil_yn_, -0.1);

  node_.param("grid_map/show_occ_time", mp_.show_occ_time_, false);
  node_.param("grid_map/pose_type", mp_.pose_type_, 1);

  node_.param("grid_map/frame_id", mp_.frame_id_, string("world"));
  node_.param("grid_map/local_map_margin", mp_.local_map_margin_, 1);
  node_.param("grid_map/ground_height", mp_.ground_height_, 1.0);

  node_.param("grid_map/odom_depth_timeout", mp_.odom_depth_timeout_, 1.0);

  node_.param("grid_map/selfbox_x", selfbox_x, 0.0);
  node_.param("grid_map/selfbox_y", selfbox_y, 0.0);
  node_.param("grid_map/selfbox_z", selfbox_z, 0.0);
  node_.param("grid_map/self_id", self_id, 0);

  if (mp_.virtual_ceil_height_ - mp_.ground_height_ > z_size)
  {
    mp_.virtual_ceil_height_ = mp_.ground_height_ + z_size;
  }

  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.unknown_flag_ = 0.01;

  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max log: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  // hit: 0.619039
  // miss: -0.619039
  // min log: -1.99243
  // max log : 2.19722
  // thresh log: 0.8472

  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

  // initialize data buffers

  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);
  cout<<"buffer_size "<<buffer_size<<endl;
  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  int inf_step_z = ceil(mp_.obstacles_inflation_z_ / mp_.resolution_);
  int inf_num = (2 * inf_step + 1) * (2 * inf_step + 1) * (2 * inf_step_z + 1);
  md_.occupancy_buffer_unknowinflate_ = vector<char>(buffer_size, inf_num);

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);

  md_.raycast_num_ = 0;

  md_.proj_points_cnt = 0;
  md_.self_pos_ << 0.0, 0.0, 0.0;
  // use odometry and point cloud
  indep_cloud_sub_ =
      node_.subscribe<sensor_msgs::PointCloud2>("grid_map/cloud", 10, &GridMap::cloudCallback, this, ros::TransportHints().tcpNoDelay()); // drone_$(arg drone_id)_pcl_render_node/cloud"
  indep_odom_sub_ =
      node_.subscribe<nav_msgs::Odometry>("grid_map/odom", 10, &GridMap::odomCallback, this, ros::TransportHints().tcpNoDelay());

  occ_inflate_sub_ =
      node_.subscribe<sensor_msgs::PointCloud2>("grid_map/occuinflateRecv", 10, &GridMap::occuinflateRecvCallback, this, ros::TransportHints().tcpNoDelay());
  master_odom_sub_ =
      node_.subscribe<nav_msgs::Odometry>("grid_map/master_odom", 10, &GridMap::masterOdomCallback, this, ros::TransportHints().tcpNoDelay());

  // other_odom_sub_ =
  //     node_.subscribe<nav_msgs::Odometry>("grid_map/others_odom", 10, &GridMap::otherOdomCallback, this);

  // occ_timer_ = node_.createTimer(ros::Duration(0.03), &GridMap::updateOccupancyCallback, this);
  vis_timer_ = node_.createTimer(ros::Duration(0.11), &GridMap::visCallback, this);//0.11

  map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy", 10);
  map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy_inflate", 10);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.has_cloud_ = false;
  md_.image_cnt_ = 0;
  md_.last_occ_update_time_.fromSec(0);

  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_fuse_time_ = 0.0;

  md_.flag_depth_odom_timeout_ = false;
  md_.flag_use_depth_fusion = false;
  md_.have_sensor_pos_ = false;

  // rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  // rand_noise2_ = normal_distribution<double>(0, 0.2);
  // random_device rd;
  // eng_ = default_random_engine(rd());
  resetBuffer();
  md_.local_bound_min_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
  md_.local_bound_max_ = Eigen::Vector3i::Zero();
}

void GridMap::resetBuffer()
{
  Eigen::Vector3d min_pos = mp_.map_min_boundary_;
  Eigen::Vector3d max_pos = mp_.map_max_boundary_;

  resetBuffer(min_pos, max_pos);

  md_.local_bound_min_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
  md_.local_bound_max_ = Eigen::Vector3i::Zero();

  md_.global_bound_min_ = md_.local_bound_min_;
  md_.global_bound_max_ = md_.local_bound_max_;
}

void GridMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{

  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  /* reset occ and dist buffer */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0; // 膨胀地图清空
      }
}

int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  int idx_ctns = toAddress(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1)
  {
    md_.cache_voxel_.push(id);
  }

  if (occ == 1)
    md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void GridMap::raycastProcess(const pcl::PointCloud<pcl::PointXYZ> &cloud)
{
  // if (md_.proj_points_.size() == 0)
  if (cloud.size() == 0)
    return;

  // cout<<"raycast process"<<endl;
  ros::Time t1, t2;

  md_.raycast_num_ += 1;

  int vox_idx;
  double length;

  // bounding box of updated region
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;

  for (const auto &pt : cloud)
  {
    // cout<<"raycast process2"<<endl;
    // pt_w = md_.proj_points_[i];
    pt_w << pt.x, pt.y, pt.z;
    // set flag for projected point
  
    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    length = (pt_w - md_.camera_pos_).norm();
    if (length <= mp_.min_ray_length_)
      continue;
    if (!isInMap(pt_w))
    {
      pt_w = closetPointInMap(pt_w, md_.camera_pos_);
      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
      }
      vox_idx = setCacheOccupancy(pt_w, 0);
    }
    else
    {
      vox_idx = setCacheOccupancy(pt_w, 1);
    }



    // raycasting between camera center and point

    if (vox_idx != INVALID_IDX)
    {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
      {
        continue;
      }
      else
      {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    raycaster.setInput(pt_w / mp_.resolution_, md_.camera_pos_ / mp_.resolution_);
    //
    // TODO because of thr inflate of obs and some other cause ,may can't clear the dynamite obs clearly
    while (raycaster.step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
      // length = (tmp - md_.camera_pos_).norm();

      // if (length < mp_.min_ray_length_) break;

      vox_idx = setCacheOccupancy(tmp, 0);
      if (vox_idx != INVALID_IDX)
      {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
        {
          break;
        }
        else
        {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }

  // min_x = min(min_x, md_.camera_pos_(0));
  // min_y = min(min_y, md_.camera_pos_(1));
  // min_z = min(min_z, md_.camera_pos_(2));

  // max_x = max(max_x, md_.camera_pos_(0));
  // max_y = max(max_y, md_.camera_pos_(1));
  // max_z = max(max_z, md_.camera_pos_(2));
  // max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);
  md_.global_bound_min_(0) = min(md_.global_bound_min_(0), md_.local_bound_min_(0));
  md_.global_bound_min_(1) = min(md_.global_bound_min_(1), md_.local_bound_min_(1));
  md_.global_bound_min_(2) = min(md_.global_bound_min_(2), md_.local_bound_min_(2));

  md_.global_bound_max_(0) = max(md_.global_bound_max_(0), md_.local_bound_max_(0));
  md_.global_bound_max_(1) = max(md_.global_bound_max_(1), md_.local_bound_max_(1));
  md_.global_bound_max_(2) = max(md_.global_bound_max_(2), md_.local_bound_max_(2));
  md_.local_updated_ = true;

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

  while (!md_.cache_voxel_.empty())
  {

    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    double log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ? mp_.prob_hit_log_ : mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    // if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
    // {
    //   continue;
    // }
    // else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
    // {
    //   md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
    //   continue;
    // }

    bool occupy_unknown_ori = md_.occupancy_buffer_[idx_ctns] < mp_.clamp_min_log_ ? true : false;
    bool occupy_ori = md_.occupancy_buffer_[idx_ctns] >= mp_.min_occupancy_log_ ? true : false;
    md_.occupancy_buffer_[idx_ctns] =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
    bool occupy = md_.occupancy_buffer_[idx_ctns] >= mp_.min_occupancy_log_ ? true : false;
    bool occupy_unknown = md_.occupancy_buffer_[idx_ctns] < mp_.clamp_min_log_ ? true : false;

    if (occupy_ori != occupy)
    {
      // cout<<"count change"<<occupy<<endl;
      if (occupy_ori == true && occupy == false) // down
        inflateCount(idx, false, md_.occupancy_buffer_inflate_);
      else
        inflateCount(idx, true, md_.occupancy_buffer_inflate_);
    }
    if ((occupy_unknown_ori || occupy_ori) && (!occupy && !occupy_unknown)) // false
    {
      // cout<<"flase update"<<endl;
      inflateCount(idx, false, md_.occupancy_buffer_unknowinflate_);
    }
    else if (!occupy_unknown_ori && !occupy_ori && (occupy || occupy_unknown))// true
    {
      inflateCount(idx, true, md_.occupancy_buffer_unknowinflate_);
    }
  }
}

void GridMap::inflateCount(const Eigen::Vector3i idx, const bool is_up, vector<char> &map) // true means  free change to occ
{
  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  int inf_step_z = ceil(mp_.obstacles_inflation_z_ / mp_.resolution_);
  // cout << inf_step << " " << inf_step_z << endl;
  Eigen::Vector3i index_inf;
  if (is_up)
  {
    for (int x = -inf_step; x <= inf_step; ++x)
      for (int y = -inf_step; y <= inf_step; ++y)
        for (int z = -inf_step_z; z <= inf_step_z; ++z)
        {
          index_inf(0) = idx(0) + x;
          index_inf(1) = idx(1) + y;
          index_inf(2) = idx(2) + z;

          if (!isInMap(index_inf))
            continue;

          int idx_inf = toAddress(index_inf);
          map[idx_inf] += 1;
        }
  }
  else
  {
    for (int x = -inf_step; x <= inf_step; ++x)
      for (int y = -inf_step; y <= inf_step; ++y)
        for (int z = -inf_step_z; z <= inf_step_z; ++z)
        {
          index_inf(0) = idx(0) + x;
          index_inf(1) = idx(1) + y;
          index_inf(2) = idx(2) + z;

          if (!isInMap(index_inf))
            continue;

          int idx_inf = toAddress(index_inf);
          map[idx_inf] -= 1;
          // if(map[idx_inf]<=0)
          // {
          //   cout<<"have free"<<endl;
          // } 
        }
  }
}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)
{
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
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

  return camera_pt + (min_t - 1e-3) * diff;
}

void GridMap::updateInflateLocalMap()
{
  /*clear outside local*/
  // Eigen::Vector3i min_vec_margin = min_vec - Eigen::Vector3i(vec_margin,
  // vec_margin, vec_margin); Eigen::Vector3i max_vec_margin = max_vec +
  // Eigen::Vector3i(vec_margin, vec_margin, vec_margin);

  //

  // Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  // Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  // boundIndex(min_cut_m);
  // boundIndex(max_cut_m);

  // clear data outside the local range

  // for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
  //   for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
  //   {

  //     for (int z = min_cut_m(2); z < min_cut(2); ++z)
  //     {
  //       int idx = toAddress(x, y, z);
  //       md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
  //     }

  //     for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z)
  //     {
  //       int idx = toAddress(x, y, z);
  //       md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
  //     }
  //   }

  // for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
  //   for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
  //   {

  //     for (int y = min_cut_m(1); y < min_cut(1); ++y)
  //     {
  //       int idx = toAddress(x, y, z);
  //       md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
  //     }

  //     for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y)
  //     {
  //       int idx = toAddress(x, y, z);
  //       md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
  //     }
  //   }

  // for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
  //   for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
  //   {

  //     for (int x = min_cut_m(0); x < min_cut(0); ++x)
  //     {
  //       int idx = toAddress(x, y, z);
  //       md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
  //     }

  //     for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x)
  //     {
  //       int idx = toAddress(x, y, z);
  //       md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
  //     }
  //   }

  // inflate occupied voxels to compensate robot size

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  int inf_step_z = 8;
  // vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
  //  inf_pts.resize(4 * inf_step + 3);
  Eigen::Vector3i inf_pt;
  Eigen::Vector3i min_idx = md_.local_bound_min_;
  Eigen::Vector3i max_idx = md_.local_bound_max_;
  // clear outdated data
  for (int x = md_.local_bound_min_(0) + inf_step; x <= md_.local_bound_max_(0) - inf_step; ++x)
    for (int y = md_.local_bound_min_(1) + inf_step; y <= md_.local_bound_max_(1) - inf_step; ++y)
      //
      for (int z = md_.global_bound_min_(2); z <= md_.global_bound_max_(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }

  // inflate obstacles
  Eigen::Vector3i now_index;
  posToIndex(md_.self_pos_, now_index);
  int inflate_z = now_index(2) + 1;
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = inflate_z; z <= inflate_z + 1; ++z)
      {
        if (md_.occupancy_buffer_[toAddress(x, y, z)] > mp_.min_occupancy_log_)
        {
          // inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);
          for (int dx = -inf_step; dx <= inf_step; ++dx)
            for (int dy = -inf_step; dy <= inf_step; ++dy)
              for (int dz = -inf_step_z; dz <= inf_step_z; ++dz)
              {
                // index in x, y, z
                inf_pt(0) = x + dx;
                inf_pt(1) = y + dy;
                inf_pt(2) = z + dz;
                // index is the 3d index of x,y,z//address is the index of one demension

                if (!isInMap(inf_pt))
                  continue;
                min_idx(0) = min(min_idx(0), inf_pt(0));
                min_idx(1) = min(min_idx(1), inf_pt(1));
                min_idx(2) = min(min_idx(2), inf_pt(2));

                max_idx(0) = max(max_idx(0), inf_pt(0));
                max_idx(1) = max(max_idx(1), inf_pt(1));
                max_idx(2) = max(max_idx(2), inf_pt(2));

                int idx_inf = toAddress(inf_pt);
                md_.occupancy_buffer_inflate_[idx_inf] = 1;
              }
        }
      }

  md_.global_bound_min_(0) = min(md_.global_bound_min_(0), min_idx(0));
  md_.global_bound_min_(1) = min(md_.global_bound_min_(1), min_idx(1));
  md_.global_bound_min_(2) = min(md_.global_bound_min_(2), min_idx(2));

  md_.global_bound_max_(0) = max(md_.global_bound_max_(0), max_idx(0));
  md_.global_bound_max_(1) = max(md_.global_bound_max_(1), max_idx(1));
  md_.global_bound_max_(2) = max(md_.global_bound_max_(2), max_idx(2));
}

void GridMap::visCallback(const ros::TimerEvent & /*event*/)
{
  publishMapInflate(true);
  publishMap();
}

bool GridMap::synCloudAndOdom()
{
  if (!md_.have_sensor_pos_)
  {
    cout << "no sensor pos" << endl;
    return false;
  }
  // ROS_WARN("diff_2");
  md_.last_occ_update_time_ = ros::Time::now();
  std::map<double, Eigen::Vector3d>::iterator it = md_.camera_pos_win_.begin();
  std::map<double, Eigen::Vector3d>::iterator min_it = it;
  double min_diff = 100.0;
  for (; it != md_.camera_pos_win_.end(); it++)
  {
    double temp_diff = it->first - md_.now_map_stamp_;
    if (fabs(min_diff) > fabs(temp_diff))
    {
      min_diff = temp_diff;
      min_it = it;
    }
  }
  // cout<<"final dirct"<<endl;
  if (min_it == md_.camera_pos_win_.begin())
  {
    if (min_diff >= 0.1)
    {
      ROS_ERROR("so old map time!%f", min_diff); // win need to be wider
      return false;
    }
  }
  min_it++;
  if (min_it == md_.camera_pos_win_.end())
  {
    if (min_diff <= -0.1)
    {
      ROS_ERROR("so old sensor odom time!");
      return false;
    }
  }
  min_it--;
  md_.camera_pos_(0) = min_it->second.x();
  md_.camera_pos_(1) = min_it->second.y();
  md_.camera_pos_(2) = min_it->second.z();
  return true;
  // ROS_WARN("syn up!");
}

void GridMap::updateOccupancyCallback(const ros::TimerEvent & /*event*/)
{
  if (md_.last_occ_update_time_.toSec() < 1.0)
    md_.last_occ_update_time_ = ros::Time::now();
  // ROS_WARN("diff_1");
  if (!md_.occ_need_update_)
  {
    return;
  }

  synCloudAndOdom();
  // cout<<"---test--"<<md_.last_occ_update_time_.toSec()<<md_.camera_pos_<<endl;
  /* update occupancy */
  // ros::Time t1, t2, t3, t4;
  // t1 = ros::Time::now();
  // t2 = ros::Time::now();
  // raycastProcess();
  // t3 = ros::Time::now();

  updateInflateLocalMap();

  // t4 = ros::Time::now();

  // cout << setprecision(7);
  // cout << "t2=" << (t2-t1).toSec() << " t3=" << (t3-t2).toSec() << " t4=" << (t4-t3).toSec() << endl;;

  // md_.fuse_time_ += (t2 - t1).toSec();
  // md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

  // if (mp_.show_occ_time_)
  //   ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
  //            md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  // cout<<"---test--"<< md_.last_occ_update_time_.toSec()<<md_.camera_pos_<<endl;
}

void GridMap::odomCallback(const nav_msgs::OdometryConstPtr &odom) // 本机在世界系下的坐标
{
  md_.self_pos_(0) = odom->pose.pose.position.x;
  md_.self_pos_(1) = odom->pose.pose.position.y;
  md_.self_pos_(2) = odom->pose.pose.position.z;

  md_.self_odom_time_ = odom->header.stamp;
  md_.has_odom_ = true;
}

// void GridMap::otherOdomCallback(const nav_msgs::OdometryConstPtr &odom)
// {
//     std::string numstr = odom -> child_frame_id.substr(4);
//     int id  = std::stoi(numstr);
//     double time = odom->header.stamp.toSec();
//     Eigen::Vector3d temp_pos;
//     temp_pos <<odom->pose.pose.position.x,
//                               odom->pose.pose.position.y,
//                               odom->pose.pose.position.z;
//   if(other_pos_buf_.size() <= id)
//   {
//       for (size_t i = other_pos_buf_.size(); i <= id; i++)
//       {
//         pair<double,Eigen::Vector3d> blank;
//         blank.first = 0;
//         other_pos_buf_.push_back(blank);
//       }
//   }
//   other_pos_buf_[id].first = time;
//   other_pos_buf_[id].second = temp_pos;
// }
void GridMap::masterOdomCallback(const nav_msgs::OdometryConstPtr &odom)
{
  double temp_time = odom->header.stamp.toSec();
  if (!md_.have_sensor_pos_)
  {
    md_.have_sensor_pos_ = true;
  }
  else
  {
    auto iter = md_.camera_pos_win_.end();
    iter--;
    if ((temp_time - iter->first) < 0.1)
      return;
  }

  Eigen::Vector3d temp_pos;
  temp_pos << odom->pose.pose.position.x,
      odom->pose.pose.position.y,
      odom->pose.pose.position.z;
  if (md_.camera_pos_win_.size() < 8)
  {
    md_.camera_pos_win_.insert(pair<double, Eigen::Vector3d>(temp_time, temp_pos));
  }
  else
  {
    md_.camera_pos_win_.erase(md_.camera_pos_win_.begin());
    md_.camera_pos_win_.insert(pair<double, Eigen::Vector3d>(temp_time, temp_pos));
  }
  // todo  interpolation to be moreaccrrate
  // md_.camera_pos_(0) = odom->pose.pose.position.x;
  // md_.camera_pos_(1) = odom->pose.pose.position.y;
  // md_.camera_pos_(2) = odom->pose.pose.position.z;
}

void GridMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &img)
{

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  md_.has_cloud_ = true;

  if (!md_.has_odom_)
  {
    std::cout << "no odom!" << std::endl;
    return;
  }

  if (latest_cloud.points.size() == 0)
    return;

  if (isnan(md_.self_pos_(0)) || isnan(md_.self_pos_(1)) || isnan(md_.self_pos_(2)))
    return;

  this->resetBuffer(md_.self_pos_ - mp_.local_update_range_,
                    md_.self_pos_ + mp_.local_update_range_);

  pcl::PointXYZ pt;
  Eigen::Vector3d p3d, p3d_inf;

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  int inf_step_z = ceil(1.0 / mp_.resolution_); // 1

  double max_x, max_y, max_z, min_x, min_y, min_z;

  min_x = mp_.map_max_boundary_(0);
  min_y = mp_.map_max_boundary_(1);
  min_z = mp_.map_max_boundary_(2);

  max_x = mp_.map_min_boundary_(0);
  max_y = mp_.map_min_boundary_(1);
  max_z = mp_.map_min_boundary_(2);

  for (size_t i = 0; i < latest_cloud.points.size(); ++i)
  {
    pt = latest_cloud.points[i];
    p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

    /* point inside update range */
    Eigen::Vector3d devi = p3d - md_.self_pos_;
    Eigen::Vector3i inf_pt;
    if (devi(2) <= 0.1 && devi(2) >= -0.1)
    {
      if (fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1) &&
          fabs(devi(2)) < mp_.local_update_range_(2))
      {
        /* inflate the point */
        for (int x = -inf_step; x <= inf_step; ++x)
          for (int y = -inf_step; y <= inf_step; ++y)
            for (int z = -inf_step_z; z <= inf_step_z; ++z)
            {

              p3d_inf(0) = pt.x + x * mp_.resolution_;
              p3d_inf(1) = pt.y + y * mp_.resolution_;
              p3d_inf(2) = pt.z + z * mp_.resolution_;

              max_x = max(max_x, p3d_inf(0));
              max_y = max(max_y, p3d_inf(1));
              max_z = max(max_z, p3d_inf(2));

              min_x = min(min_x, p3d_inf(0));
              min_y = min(min_y, p3d_inf(1));
              min_z = min(min_z, p3d_inf(2));

              posToIndex(p3d_inf, inf_pt); // index is the 3d index of x,y,z//address is the index of one demension

              if (!isInMap(inf_pt))
                continue;

              int idx_inf = toAddress(inf_pt);

              md_.occupancy_buffer_inflate_[idx_inf] = 1;
            }
      }
    }
  }

  min_x = min(min_x, md_.self_pos_(0));
  min_y = min(min_y, md_.self_pos_(1));
  min_z = min(min_z, md_.self_pos_(2));

  max_x = max(max_x, md_.self_pos_(0));
  max_y = max(max_y, md_.self_pos_(1));
  max_z = max(max_z, md_.self_pos_(2));

  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.global_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.global_bound_min_);

  boundIndex(md_.global_bound_min_);
  boundIndex(md_.global_bound_max_);

  // // add virtual ceiling to limit flight height
  // if (mp_.virtual_ceil_height_ > -0.5) {
  //   int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_) - 1;
  //   for (int x = md_.global_bound_min_(0); x <= md_.global_bound_max_(0); ++x)
  //     for (int y = md_.global_bound_min_(1); y <= md_.global_bound_max_(1); ++y) {
  //       md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
  //     }
  // }
}

// if define a more simple msg than pointcloud2 will use less commu band????
// consider the dead area
void GridMap::occuinflateRecvCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  std::string numstr = msg ->header.frame_id.substr(4);
  int id  = std::stoi(numstr);
  if(id != self_id) return;
  md_.proj_points_cnt = 0;
  md_.proj_points_.clear();
  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*msg, latest_cloud);
  // ros::Time start_time = ros::Time::now();
  // cout<< "size" << latest_cloud.points.size() <<endl;

  md_.now_map_stamp_ = msg->header.stamp.toSec();

  if (synCloudAndOdom())
  {
    raycastProcess(latest_cloud);
  }

  // for (const auto&pt:cloud)
  // {
  //   double dx = pt.x - md_.self_pos_.x();
  //   double dy = pt.y - md_.self_pos_.y();
  //   double dz = pt.z - md_.self_pos_.z();
  //   if (fabs(dx) <= selfbox_x && fabs(dy) <= selfbox_y && fabs(dz) <= selfbox_z)
  //   {
  //     continue;
  //   }
  //   bool  erase_flag = false;
  //   for(auto it = other_pos_buf_.begin(); it!= other_pos_buf_.end(); it++)
  //   {
  //     if(fabs(it->first - md_.now_map_stamp_ ) > 1.0)
  //       continue;
  //     double dx1 = pt.x - it->second.x();
  //     double dy1 = pt.y - it->second.y();
  //     double dz1 = pt.z - it->second.z();
  //     if (fabs(dx1) <= selfbox_x && fabs(dy1) <= selfbox_y && fabs(dz1) <= selfbox_z)
  //     {
  //       erase_flag = true;
  //       break;
  //     }
  //   }
  //   if(erase_flag) continue;
  //   md_.proj_points_.push_back( Eigen::Vector3d(pt.x, pt.y, pt.z));
  //   md_.proj_points_cnt ++;
  // }

  if (!md_.occ_need_update_)
  {
    md_.occ_need_update_ = true;
  }
  // double time = ros::Time::now().toSec();
  // std::cout<<"---timecost1---"<<cost_time<<"---"<<std::endl;
  // last_time = time;
}

//
void GridMap::publishMap()
{

  if (map_pub_.getNumSubscribers() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.global_bound_min_;
  Eigen::Vector3i max_cut = md_.global_bound_max_;

  // int lmm = mp_.local_map_margin_ / 2;
  // min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
  // max_cut += Eigen::Vector3i(lmm, lmm, lmm);

  boundIndex(min_cut);
  boundIndex(max_cut);

  int all_count = 0;

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        all_count++;
        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.min_occupancy_log_)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_.publish(cloud_msg);
}

void GridMap::publishMapInflate(bool all_info)
{

  if (map_inf_pub_.getNumSubscribers() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.global_bound_min_;
  Eigen::Vector3i max_cut = md_.global_bound_max_;

  // if (all_info)
  // {
  //   int lmm = mp_.local_map_margin_;
  //   min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
  //   max_cut += Eigen::Vector3i(lmm, lmm, lmm);
  // }

  
   
  boundIndex(min_cut);
  boundIndex(max_cut);

  int count = 0; int all_count = 0;
  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        all_count++;
        if (md_.occupancy_buffer_unknowinflate_[toAddress(x, y, z)] <= 0)
        {
          count++;
          continue;
        }

        // if (md_.occupancy_buffer_unknowinflate_[toAddress(x, y, z)] <= 0)
        //   continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_)
          continue;
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_.publish(cloud_msg);

  // ROS_INFO("pub map");
}

bool GridMap::odomValid() { return md_.has_odom_; }

bool GridMap::hasDepthObservation() { return md_.has_first_depth_; }

Eigen::Vector3d GridMap::getOrigin() { return mp_.map_origin_; }

// int GridMap::getVoxelNum() {
//   return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
// }

void GridMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = mp_.map_origin_, size = mp_.map_size_;
}
