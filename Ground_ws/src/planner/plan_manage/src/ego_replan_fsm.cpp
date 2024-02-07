#include <plan_manage/ego_replan_fsm.h>
// #include "ego_replan_fsm.h"

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = true;
    wp_id_ = 0;

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1); // 2
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/planning_horizen_time", planning_horizen_time_, -1.0);
    nh.param("fsm/emergency_time", emergency_time_, 1.0);
    nh.param("fsm/realworld_experiment", flag_realworld_experiment_, false);
    nh.param("fsm/fail_safe", enable_fail_safe_, true);

    // have_trigger_ = !flag_realworld_experiment_;

    have_trigger_ = false;
    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new EGOPlannerManager);
    planner_manager_->initPlanModules(nh, visualization_);
    planner_manager_->deliverTrajToOptimizer(); // store trajectories
    planner_manager_->deliverMasterTrajToOptimizer();

    if (target_type_ == TARGET_TYPE::FORMATION)
    {
      std::vector<double> pos;
      nh.getParam("formation/drone" + to_string(planner_manager_->pp_.drone_id), pos);
      formation_pos_ << pos[0], pos[1], pos[2];
    }

    collision_detect_.init(nh);
    predict_timer_ = nh.createTimer(ros::Duration(0.2), &EGOReplanFSM::collisionPredictCallback, this);

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("odom_world", 1, &EGOReplanFSM::odometryCallback, this, ros::TransportHints().tcpNoDelay());

    // if (planner_manager_->pp_.drone_id >= 1)
    // {
    //   string sub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id - 1) + string("_planning/swarm_trajs");
    //   swarm_trajs_sub_ = nh.subscribe(sub_topic_name.c_str(), 10, &EGOReplanFSM::swarmTrajsCallback, this, ros::TransportHints().tcpNoDelay());
    // }
    // string pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string("_planning/swarm_trajs");
    // swarm_trajs_pub_ = nh.advertise<traj_utils::MultiBsplines>(pub_topic_name.c_str(), 10);

    // swarm_trajs vs broadcast_bspline???
    // 以下两个都映射为了 /broadcast_bspline
    broadcast_bspline_pub_ = nh.advertise<traj_utils::Bspline>("planning/broadcast_bspline_from_planner", 10);
    broadcast_bspline_sub_ = nh.subscribe("planning/broadcast_bspline_to_planner", 100, &EGOReplanFSM::BroadcastBsplineCallback, this, ros::TransportHints().tcpNoDelay());

    // broadcast_odom_sub_ = nh.subscribe("broadcast_odom_receive", 10, &EGOReplanFSM::BroadcastOdomCallback, this, ros::TransportHints().tcpNoDelay());

    // 发布给traj_server
    bspline_pub_ = nh.advertise<traj_utils::Bspline>("planning/bspline", 10);
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
      waypoint_sub_ = nh.subscribe("/goal", 1, &EGOReplanFSM::waypointCallback, this, ros::TransportHints().tcpNoDelay());
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &EGOReplanFSM::triggerCallback, this, ros::TransportHints().tcpNoDelay());

      ROS_INFO("Wait for 1 second.");
      int count = 0;
      while (ros::ok() && count++ < 1000)
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }

      ROS_WARN("Waiting for trigger from [n3ctrl] from RC");

      while (ros::ok() && (!have_odom_ || !have_trigger_))
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }

      readGivenWps();
    }
    else if (target_type_ == TARGET_TYPE::TRACKING)
    {
      master_odom_sub_ = nh.subscribe("/odom3_topic", 1, &EGOReplanFSM::realWaypointCallback, this, ros::TransportHints().tcpNoDelay());
      trigger_sub_ = nh.subscribe("/goal", 1, &EGOReplanFSM::triggerCallback, this, ros::TransportHints().tcpNoDelay());
    }
    else if (target_type_ == TARGET_TYPE::FORMATION)
    {
      master_odom_sub_ = nh.subscribe("formation/master_odom", 1, &EGOReplanFSM::realWaypointCallback, this, ros::TransportHints().tcpNoDelay());
      master_traj_sub = nh.subscribe("formation/master_traj_planner", 10, &EGOReplanFSM::masterBsplineCallback, this, ros::TransportHints().tcpNoDelay());
      waypoint_sub_ = nh.subscribe("/goal", 1, &EGOReplanFSM::waypointCallback, this, ros::TransportHints().tcpNoDelay());
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }

  void EGOReplanFSM::readGivenWps()
  {
    if (waypoint_num_ <= 0)
    {
      ROS_ERROR("Wrong waypoint_num_ = %d", waypoint_num_);
      return;
    }

    wps_.resize(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps_[i](0) = waypoints_[i][0];
      wps_[i](1) = waypoints_[i][1];
      wps_[i](2) = waypoints_[i][2];

      // end_pt_ = wps_.back();
    }

    // bool success = planner_manager_->planGlobalTrajWaypoints(
    //   odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    //   wps_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    // plan first global waypoint
    wp_id_ = 0;
    planNextWaypoint(wps_[wp_id_]);

    // if (success)
    // {

    //   /*** display ***/
    //   constexpr double step_size_t = 0.1;
    //   int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    //   std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    //   for (int i = 0; i < i_end; i++)
    //   {
    //     gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    //   }

    //   end_vel_.setZero();
    //   have_target_ = true;
    //   have_new_target_ = true;

    //   /*** FSM ***/
    //   // if (exec_state_ == WAIT_TARGET)
    //   //changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    //   // trigger_ = true;
    //   // else if (exec_state_ == EXEC_TRAJ)
    //   //   changeFSMExecState(REPLAN_TRAJ, "TRIG");

    //   // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
    //   ros::Duration(0.001).sleep();
    //   visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    //   ros::Duration(0.001).sleep();
    // }
    // else
    // {
    //   ROS_ERROR("Unable to generate global trajectory!");
    // }
  }

  void EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
  {
    bool success = false;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), next_wp, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    // visualization_->displayGoalPoint(next_wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      end_pt_ = next_wp;

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else
      {
        while (exec_state_ != EXEC_TRAJ)
        {
          ros::spinOnce();
          ros::Duration(0.001).sleep();
        }
        changeFSMExecState(REPLAN_TRAJ, "TRIG");
      }

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");
    }
  }

  void EGOReplanFSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    have_trigger_ = true;
    cout << "Triggered!" << endl;
    init_pt_ = odom_pos_;
  }

  void EGOReplanFSM::realWaypointCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    if (!have_master_odom_)
      have_master_odom_ = true;
    master_odom_pos_ << msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z;
    if (target_type_ == TARGET_TYPE::TRACKING)
    {
      if ((msg->header.stamp.toSec() - last_waypoint_time_.toSec()) < 0.3)
        return;
      if (!have_trigger_)
      {
        return;
      }
      last_waypoint_time_ = msg->header.stamp;

      cout << "way point Triggered!" << endl;
      // trigger_ = true;
      init_pt_ = odom_pos_;

      Eigen::Vector3d p1;
      if (planner_manager_->pp_.drone_id == 2)
      {
        p1 << 0.5, 1.5, 0;
      }
      else if (planner_manager_->pp_.drone_id == 3)
      {
        p1 << -0.5, 1.5, 0;
      }
      Eigen::Quaterniond rot(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
      p1 = rot * p1 + master_odom_pos_;
      waypoints_[0][0] = p1.x();
      waypoints_[0][1] = p1.y();
      waypoints_[0][2] = odom_pos_(2);
      Eigen::Vector3d end_wp(p1.x(), p1.y(), odom_pos_(2));

      planNextWaypoint(end_wp);
    }
  }

  void EGOReplanFSM::waypointCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    if (target_type_ == TARGET_TYPE::FORMATION)
    {
      if (!have_master_odom_)
      {
        ROS_WARN("wait master odom!");
        return;
      }
      final_set_ = false;
      Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, 1.0);
      // Eigen::Vector3d dir = (end_wp - master_odom_pos_).normalized();
      // formation_end_ = end_wp + Eigen::Vector3d(dir(0) * formation_pos_(0) - dir(1) * formation_pos_(1),
      //                                   dir(1) * formation_pos_(0) + dir(0) * formation_pos_(1),
      //                                   formation_pos_(2));
      formation_end_ = end_wp;
      last_waypoint_time_ = ros::Time::now();
    }
    else
    {
      if (msg->pose.position.z > 0)
        return;
      else
      {
        cout << "Triggered!" << endl;
        // trigger_ = true;
        waypoints_[0][0] = msg->pose.position.x;
        waypoints_[0][1] = msg->pose.position.y;
        waypoints_[0][2] = odom_pos_(2);
        init_pt_ = odom_pos_;

        Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, odom_pos_(2));

        planNextWaypoint(end_wp);
      }
    }
  }

  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z; // msg->pose.pose.position.z;//

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = 0; // msg->twist.twist.linear.z;

    // odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    odom_cov_ = Eigen::Map<const Eigen::Matrix<double, 6, 6>>(msg->pose.covariance.data());

    have_odom_ = true; // car 2
  }

  void EGOReplanFSM::BroadcastOdomCallback(const nav_msgs::OdometryPtr &msg)
  {
    // cout<<"here"<<endl;
    std::string numstr = msg->child_frame_id.substr(4);
    int id = std::stoi(numstr);
    // cout<<"here"<<endl;
    if (id == planner_manager_->pp_.drone_id)
      return;

    if (abs((ros::Time::now() - msg->header.stamp).toSec()) > 0.50)
    {
      // ROS_ERROR("Time difference of odom is too large! Local - Remote Agent %d = %fs",
      //         id, (ros::Time::now() - msg->header.stamp).toSec());
      return;
    }

    if (planner_manager_->swarm_odoms_buf_.size() <= id)
    {
      for (size_t i = planner_manager_->swarm_odoms_buf_.size(); i <= id; i++)
      {
        OneIDodomOfSwarm blank;
        blank.drone_id = -1;
        planner_manager_->swarm_odoms_buf_.push_back(blank);
      }
    }
    planner_manager_->swarm_odoms_buf_[id].drone_id = id;
    planner_manager_->swarm_odoms_buf_[id].pose(0) = msg->pose.pose.position.x;
    planner_manager_->swarm_odoms_buf_[id].pose(1) = msg->pose.pose.position.y;
    planner_manager_->swarm_odoms_buf_[id].pose(2) = odom_pos_(2);
    if (exec_state_ == EXEC_TRAJ)
    {
      if ((planner_manager_->swarm_odoms_buf_[id].pose - odom_pos_).norm() < 0.20)
      {
        changeFSMExecState(EMERGENCY_STOP, "ODOM_CHECK");
        cout << "happened" << endl;
      }
      else
      {
        // cout<<"not happened"<<endl;
      }
    }
    else
    {
      // cout<<"other condition"<<endl;
    }
  }

  void EGOReplanFSM::collisionPredictCallback(const ros::TimerEvent &e)
  {
    if (!have_odom_)
    {
      return;
    }
    if (exec_state_ != REPLAN_TRAJ && exec_state_ != EXEC_TRAJ)
    {
      return;
    }
    // if
    // 在此处检测碰撞
    auto info = &planner_manager_->local_data_;
    ros::Time t1 = ros::Time::now();
    double colli_time = collision_detect_.caculate_collision(planner_manager_->grid_map_, info->position_traj_, info->start_time_,
                                                             odom_pos_, odom_vel_, odom_orient_, odom_cov_);
    ros::Time t2 = ros::Time::now();
    // ROS_WARN("calcaulate collision time %f",(t2-t1).toSec());
    if (colli_time < emergency_time_ && colli_time >= 0)
    {
      changeFSMExecState(EMERGENCY_STOP, "COL_PRD");
    }

    t1 = ros::Time::now();
    collision_detect_.visualize_collision();
    t2 = ros::Time::now();
    // ROS_WARN("calcaulate visual time %f",(t2-t1).toSec());
  }

  void EGOReplanFSM::masterBsplineCallback(const traj_utils::BsplinePtr &msg)
  {
    // if((ros::Time::now() - last_waypoint_time_).toSec() > 3 && formation_count_ ==0)
    // {
    //     // formation_count_ ++;
    //     cout<<"change formation"<<endl;
    //     if(planner_manager_->pp_.drone_id == 2)
    //     formation_pos_<<-2,-2,0;
    //   else
    //     formation_pos_<<-2.0,2,0;
    //   last_waypoint_time_ = ros::Time::now();
    // }
    // else if ((ros::Time::now() - last_waypoint_time_).toSec() > 2 && formation_count_ == 1)
    // {
    //   formation_count_ ++;
    //   if(planner_manager_->pp_.drone_id == 2)
    //     formation_pos_<<-2,-2,0;
    //   else
    //     formation_pos_<<-2.0,2,0;
    //   last_waypoint_time_ = ros::Time::now();
    // }
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
    UniformBspline pos_traj(pos_pts, msg->order, msg->knots[1] - msg->knots[0]);
    pos_traj.setKnot(knots);
    planner_manager_->master_traj_buf_.position_traj_ = pos_traj;
    planner_manager_->master_traj_buf_.start_pos_ = pos_traj.evaluateDeBoorT(0);
    planner_manager_->master_traj_buf_.start_time_ = msg->start_time;
    double traj_duration_ = pos_traj.getTimeSum();
    planner_manager_->master_traj_buf_.duration_ = traj_duration_;
    Eigen::Vector3d pos = pos_traj.evaluateDeBoorT(traj_duration_);
    if (first_get_master_traj_)
    {
      if (have_master_odom_)
      {
        previous_local_target_ = master_odom_pos_;
        first_get_master_traj_ = false;
      }
    }
    else if ((pos - previous_local_target_).norm() > 5.0 && (pos - formation_end_).norm() > 3.0)
    {
      cout << "new target" << endl;
    }
    else if ((pos - formation_end_).norm() <= 3.0)
    {
      if (!final_set_)
      {
        cout << "form close to end" << endl;
        pos = formation_end_;
        final_set_ = true;
      }
      else
      {
        return;
      }
    }
    else
    {
      return;
    }
    Eigen::Vector3d dir = pos - master_odom_pos_;
    dir.z() = 0;
    dir = dir.normalized();

    Eigen::Vector3d end_wp;
    // visibility
    if (!final_set_)
    {
      end_wp = pos;
    }
    else
    {
      end_wp = pos + Eigen::Vector3d(dir(0) * formation_pos_(0) - dir(1) * formation_pos_(1),
                                     dir(1) * formation_pos_(0) + dir(0) * formation_pos_(1),
                                     formation_pos_(2));
    }
    end_wp(2) = odom_pos_(2);
    planNextWaypoint(end_wp);

    // planner_manager_->serFormationToOpt(dir, formation_pos_);
    previous_local_target_ = pos;
  }

  void EGOReplanFSM::BroadcastBsplineCallback(const traj_utils::BsplinePtr &msg)
  {
    size_t id = msg->drone_id;
    if ((int)id == planner_manager_->pp_.drone_id)
      return;

    if (abs((ros::Time::now() - msg->start_time).toSec()) > 0.25)
    {
      // ROS_ERROR("Time difference is too large! Local - Remote Agent %d = %fs",
      //         msg->drone_id, (ros::Time::now() - msg->start_time).toSec());
      return;
    }

    /* Fill up the buffer */
    if (planner_manager_->swarm_trajs_buf_.size() <= id)
    {
      for (size_t i = planner_manager_->swarm_trajs_buf_.size(); i <= id; i++)
      {
        OneTrajDataOfSwarm blank;
        blank.drone_id = -1;
        planner_manager_->swarm_trajs_buf_.push_back(blank);
      }
    }

    if (!have_recv_pre_agent_ && id <= planner_manager_->pp_.drone_id)
    {
      have_recv_pre_agent_ = true;
    }
    /* Test distance to the agent */
    Eigen::Vector3d cp0(msg->pos_pts[0].x, msg->pos_pts[0].y, msg->pos_pts[0].z);
    Eigen::Vector3d cp1(msg->pos_pts[1].x, msg->pos_pts[1].y, msg->pos_pts[1].z);
    Eigen::Vector3d cp2(msg->pos_pts[2].x, msg->pos_pts[2].y, msg->pos_pts[2].z); // pos_pts有三组是什么？
    Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
    if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
    {
      planner_manager_->swarm_trajs_buf_[id].drone_id = -1;
      return; // if the current drone is too far to the received agent.
    }

    /* Store data */
    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t j = 0; j < msg->knots.size(); ++j)
    {
      knots(j) = msg->knots[j];
    }
    for (size_t j = 0; j < msg->pos_pts.size(); ++j)
    {
      pos_pts(0, j) = msg->pos_pts[j].x;
      pos_pts(1, j) = msg->pos_pts[j].y;
      pos_pts(2, j) = msg->pos_pts[j].z;
    }

    planner_manager_->swarm_trajs_buf_[id].drone_id = id;

    if (msg->order % 2) // order代表b样条阶数  knots代表什么
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = msg->knots[msg->knots.size() - ceil(cutback)];
    }
    else
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = (msg->knots[msg->knots.size() - floor(cutback)] + msg->knots[msg->knots.size() - ceil(cutback)]) / 2;
    }

    UniformBspline pos_traj(pos_pts, msg->order, msg->knots[1] - msg->knots[0]);
    pos_traj.setKnot(knots);
    planner_manager_->swarm_trajs_buf_[id].position_traj_ = pos_traj;

    planner_manager_->swarm_trajs_buf_[id].start_pos_ = planner_manager_->swarm_trajs_buf_[id].position_traj_.evaluateDeBoorT(0);

    planner_manager_->swarm_trajs_buf_[id].start_time_ = msg->start_time;
    // planner_manager_->swarm_trajs_buf_[id].start_time_ = ros::Time::now(); // Un-reliable time sync

    /* Check Collision */
    if (planner_manager_->checkCollision(id))
    {
      changeFSMExecState(REPLAN_TRAJ, "TRAJ_CHECK");
    }
  }

  void EGOReplanFSM::swarmTrajsCallback(const traj_utils::MultiBsplinesPtr &msg) // 与上一个单轨迹的作用的具体区别是什么????
  {

    multi_bspline_msgs_buf_.traj.clear();
    multi_bspline_msgs_buf_ = *msg;

    // cout << "\033[45;33mmulti_bspline_msgs_buf.drone_id_from=" << multi_bspline_msgs_buf_.drone_id_from << " multi_bspline_msgs_buf_.traj.size()=" << multi_bspline_msgs_buf_.traj.size() << "\033[0m" << endl;

    if (!have_odom_)
    {
      ROS_ERROR("swarmTrajsCallback(): no odom!, return.");
      return;
    }

    if ((int)msg->traj.size() != msg->drone_id_from + 1) // drone_id must start from 0
    {
      ROS_ERROR("Wrong trajectory size! msg->traj.size()=%d, msg->drone_id_from+1=%d", (int)msg->traj.size(), msg->drone_id_from + 1);
      return;
    }

    if (msg->traj[0].order != 3) // only support B-spline order equals 3.
    {
      ROS_ERROR("Only support B-spline order equals 3.");
      return;
    }

    // Step 1. receive the trajectories
    planner_manager_->swarm_trajs_buf_.clear();
    planner_manager_->swarm_trajs_buf_.resize(msg->traj.size());

    for (size_t i = 0; i < msg->traj.size(); i++)
    {

      Eigen::Vector3d cp0(msg->traj[i].pos_pts[0].x, msg->traj[i].pos_pts[0].y, msg->traj[i].pos_pts[0].z);
      Eigen::Vector3d cp1(msg->traj[i].pos_pts[1].x, msg->traj[i].pos_pts[1].y, msg->traj[i].pos_pts[1].z);
      Eigen::Vector3d cp2(msg->traj[i].pos_pts[2].x, msg->traj[i].pos_pts[2].y, msg->traj[i].pos_pts[2].z);
      Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
      if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
      {
        planner_manager_->swarm_trajs_buf_[i].drone_id = -1;
        continue;
      }

      Eigen::MatrixXd pos_pts(3, msg->traj[i].pos_pts.size());
      Eigen::VectorXd knots(msg->traj[i].knots.size());
      for (size_t j = 0; j < msg->traj[i].knots.size(); ++j)
      {
        knots(j) = msg->traj[i].knots[j];
      }
      for (size_t j = 0; j < msg->traj[i].pos_pts.size(); ++j)
      {
        pos_pts(0, j) = msg->traj[i].pos_pts[j].x;
        pos_pts(1, j) = msg->traj[i].pos_pts[j].y;
        pos_pts(2, j) = msg->traj[i].pos_pts[j].z;
      }

      planner_manager_->swarm_trajs_buf_[i].drone_id = i;

      if (msg->traj[i].order % 2)
      {
        double cutback = (double)msg->traj[i].order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)];
      }
      else
      {
        double cutback = (double)msg->traj[i].order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = (msg->traj[i].knots[msg->traj[i].knots.size() - floor(cutback)] + msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)]) / 2;
      }

      // planner_manager_->swarm_trajs_buf_[i].position_traj_ =
      UniformBspline pos_traj(pos_pts, msg->traj[i].order, msg->traj[i].knots[1] - msg->traj[i].knots[0]);
      pos_traj.setKnot(knots);
      planner_manager_->swarm_trajs_buf_[i].position_traj_ = pos_traj;

      planner_manager_->swarm_trajs_buf_[i].start_pos_ = planner_manager_->swarm_trajs_buf_[i].position_traj_.evaluateDeBoorT(0);

      planner_manager_->swarm_trajs_buf_[i].start_time_ = msg->traj[i].start_time;
    }

    have_recv_pre_agent_ = true;
  }

  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++; // 状态的连续保持次数什么作用
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] /*<<" local target:"<<local_target_pt_*/ << endl;
  }

  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); // To avoid blockage

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!have_target_)
        cout << "wait for goal or trigger." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
      case INIT:
      {
        if (!have_odom_)
        {
          goto force_return;
          // return;
        }
        changeFSMExecState(WAIT_TARGET, "FSM");
        break;
      }

      case WAIT_TARGET:
      {
        if (!have_target_ || !have_trigger_)
          goto force_return;
        // return;
        else
        {
          // if ( planner_manager_->pp_.drone_id <= 0 )
          // {
          //   changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          // }
          // else
          // {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
          // }
        }
        break;
      }

      case SEQUENTIAL_START: // for swarm
      {
        // cout << "id=" << planner_manager_->pp_.drone_id << " have_recv_pre_agent_=" << have_recv_pre_agent_ << endl;
        if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
        {
          if (have_odom_ && have_target_ && have_trigger_)
          {
            bool success = planFromGlobalTraj(10); // zx-todo
            if (success)
            {
              changeFSMExecState(EXEC_TRAJ, "FSM");

              publishSwarmTrajs(false);
            }
            else
            {
              ROS_ERROR("Failed to generate the first trajectory!!!");
              changeFSMExecState(SEQUENTIAL_START, "FSM");
            }
          }
          else
          {
            ROS_ERROR("No odom or no target! have_odom_=%d, have_target_=%d", have_odom_, have_target_);
          }
        }

        break;
      }

      case GEN_NEW_TRAJ:
      {

        // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
        // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
        // start_yaw_(1) = start_yaw_(2) = 0.0;

        bool success = planFromGlobalTraj(10); // zx-todo
        if (success)
        {
          // todo there is collision checked but have publish the traj
          changeFSMExecState(EXEC_TRAJ, "FSM");
          flag_escape_emergency_ = true; // 应急标志位
          publishSwarmTrajs(false);      // false
        }
        else
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
        break;
      }

      case REPLAN_TRAJ:
      {

        if (planFromCurrentTraj(1))
        {
          changeFSMExecState(EXEC_TRAJ, "FSM");
          publishSwarmTrajs(false); // false
        }
        else
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }

        break;
      }

      case EXEC_TRAJ:
      {
        /* determine if need to replan */
        LocalTrajData *info = &planner_manager_->local_data_;
        ros::Time time_now = ros::Time::now();
        double t_cur = (time_now - info->start_time_).toSec();
        t_cur = min(info->duration_, t_cur);

        Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur); // 局部规划的现在的点
        // end_pt是全局的终点？local_target_pt_局部规划的终点
        /* && (end_pt_ - pos).norm() < 0.5 */
        if ((target_type_ == TARGET_TYPE::PRESET_TARGET) && // 是预先设置目标模式
            (wp_id_ < waypoint_num_ - 1) &&                 // 还有路标点没到
            (end_pt_ - pos).norm() < no_replan_thresh_)
        {
          wp_id_++;
          planNextWaypoint(wps_[wp_id_]);
        }
        else if ((local_target_pt_ - end_pt_).norm() < 1e-3) // close to the local target- sqrt(local_target_pt_(2) - end_pt_(2))
        {
          if (t_cur > info->duration_ - 1e-2) // 当前局部轨迹的执行时间已经超过了局部轨迹预计的执行时间，
          {
            have_target_ = false;
            have_trigger_ = false;

            if (target_type_ == TARGET_TYPE::PRESET_TARGET)
            {
              wp_id_ = 0;
              planNextWaypoint(wps_[wp_id_]);
            }
            // change last
            else if (target_type_ == TARGET_TYPE::TRACKING)
            {
              have_trigger_ = true;
            }

            final_set_ = false;
            changeFSMExecState(WAIT_TARGET, "FSM");
            goto force_return;
            // return;
          }
          else if ((end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_)
          {
            changeFSMExecState(REPLAN_TRAJ, "FSM");
          }
        }
        else if (t_cur > replan_thresh_)
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }

        break;
      }

      case EMERGENCY_STOP:
      {

        if (flag_escape_emergency_) // Avoiding repeated calls
        {
          callEmergencyStop(odom_pos_);
        }
        else
        {
          if (enable_fail_safe_ && odom_vel_.norm() < 0.1)
            changeFSMExecState(GEN_NEW_TRAJ, "FSM"); // todo: repeated colli check in gen new traj. if traj is clear safe or danger to the gen new;
          else
            callEmergencyStop(odom_pos_);
        }

        flag_escape_emergency_ = false;
        break;
      }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);

  force_return:;
    exec_timer_.start();
  }
  //
  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) // zx-todo
  {
    // 设置起始位置速度为当前无人机的位置和速度。
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    bool flag_random_poly_init;
    if (timesOfConsecutiveStateCalls().first == 1) // 如果是新切换到当前状态
      flag_random_poly_init = false;
    else
      flag_random_poly_init = true;

    for (int i = 0; i < trial_times; i++)
    {
      if (callReboundReplan(true, flag_random_poly_init))
      {
        return true;
      }
    }
    return false;
  }

  bool EGOReplanFSM::planFromCurrentTraj(const int trial_times /*=1*/)
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();

    // cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;
    // 起始位置速度为当前时刻局部轨迹计算出的位置和速度
    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      // changeFSMExecState(EXEC_TRAJ, "FSM");
      if (!success)
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = callReboundReplan(true, true);
          if (success)
            break;
        }
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }

  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)
      return;

    /* ---------- check lost of depth ---------- */
    // if (map->getOdomDepthTimeout())
    // {
    //   ROS_ERROR("Depth Lost! EMERGENCY_STOP");
    //   enable_fail_safe_ = false;
    //   changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    // }

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur_global = ros::Time::now().toSec();
    double t_start = info->start_time_.toSec();
    double t_cur = t_cur_global - t_start;
    Eigen::Vector3d p_cur = info->position_traj_.evaluateDeBoorT(t_cur);
    const double CLEARANCE = 1.0 * planner_manager_->getSwarmClearance();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur > t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      Eigen::Vector3d p_check = info->position_traj_.evaluateDeBoorT(t);
      bool occ = false;
      occ |= map->getInflateOccupancy(p_check);

      for (size_t id = 0; id < planner_manager_->swarm_trajs_buf_.size(); id++)
      {
        if ((planner_manager_->swarm_trajs_buf_.at(id).drone_id != (int)id) || (planner_manager_->swarm_trajs_buf_.at(id).drone_id == planner_manager_->pp_.drone_id))
        {
          continue;
        }

        double t_X = t + t_start - planner_manager_->swarm_trajs_buf_.at(id).start_time_.toSec();
        if (t_X > 0 && t_X < planner_manager_->swarm_trajs_buf_.at(id).duration_)
        {
          Eigen::Vector3d swarm_pridicted = planner_manager_->swarm_trajs_buf_.at(id).position_traj_.evaluateDeBoorT(t_X);
          double dist = (p_check - swarm_pridicted).norm();
          if (dist < CLEARANCE)
          {
            occ = true;
            break;
          }
        }
      }

      if (occ)
      {

        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          publishSwarmTrajs(false); // false
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            // ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY"); // car1
                                                       // changeFSMExecState(GEN_NEW_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();

    bool plan_and_refine_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    // reboundReplan规划
    have_new_target_ = false;

    cout << "refine_success=" << plan_and_refine_success << endl;
    // 发布新规划出的局部轨迹
    if (plan_and_refine_success)
    {

      auto info = &planner_manager_->local_data_;

      double colli_time = collision_detect_.caculate_collision(planner_manager_->grid_map_, info->position_traj_, info->start_time_,
                                                               odom_pos_, odom_vel_, odom_orient_, odom_cov_);
      collision_detect_.visualize_collision();
      if (colli_time < emergency_time_ && colli_time >= 0)
      {
        changeFSMExecState(EMERGENCY_STOP, "COL_PRD");
        return false;
      }

      traj_utils::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      // cout << knots.transpose() << endl;
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      /* 1. publish traj to traj_server */
      bspline_pub_.publish(bspline);

      /* 2. publish traj to the next drone of swarm */

      /* 3. publish traj for visualization */
      visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
    }

    return plan_and_refine_success;
  }

  // 一定发送单无人机轨迹
  void EGOReplanFSM::publishSwarmTrajs(bool startup_pub)
  {
    auto info = &planner_manager_->local_data_;

    traj_utils::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.drone_id = planner_manager_->pp_.drone_id;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    // cout << knots.transpose() << endl;
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    // 只有当startup_pub为ture 时，也就是SEQUENTIAL START时 才发布多无人机轨迹
    if (startup_pub)
    {
      multi_bspline_msgs_buf_.drone_id_from = planner_manager_->pp_.drone_id; // zx-todo   设置为当前无人机
      if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id + 1)
      {
        multi_bspline_msgs_buf_.traj.back() = bspline;
      }
      else if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id)
      {
        multi_bspline_msgs_buf_.traj.push_back(bspline);
      }
      else
      {
        ROS_ERROR("Wrong traj nums and drone_id pair!!! traj.size()=%d, drone_id=%d", (int)multi_bspline_msgs_buf_.traj.size(), planner_manager_->pp_.drone_id);
        // return plan_and_refine_success;
      }
      swarm_trajs_pub_.publish(multi_bspline_msgs_buf_); // 发布multi_bspline_msgs_buf_
    }

    broadcast_bspline_pub_.publish(bspline); // 当前无人机此刻的局部B样条轨迹。
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    traj_utils::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_.publish(bspline);

    return true;
  }
  // 在全局轨迹planner_manager_->global_data_ 找到距离当前无人机位置planning_horizen_ 处的点
  // 作为局部轨迹的目标点，
  // 并将全局轨迹上离当前位置最近的点对应的时间赋值给
  // planner_manager_->global_data_.last_progress_time_ 。
  void EGOReplanFSM::getLocalTarget()
  {
    double t;
    // planning_horizen_//局部规划的范围
    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;
    // 这个循环是在这个范围内对应的时间步长里寻找最小的距离
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
      {
        // Important cornor case!
        for (; t < planner_manager_->global_data_.global_duration_; t += t_step)
        {
          Eigen::Vector3d pos_t_temp = planner_manager_->global_data_.getPosition(t);
          double dist_temp = (pos_t_temp - start_pt_).norm();
          if (dist_temp < planning_horizen_)
          {
            pos_t = pos_t_temp;
            dist = (pos_t - start_pt_).norm();
            cout << "Escape cornor case \"getLocalTarget\"" << endl;
            break;
          }
        }
      }

      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }

      if (dist >= planning_horizen_)
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }
    // 如果局部轨迹的目标点对应的时间 > 全局轨迹的时长
    // 那么将全局终点作为局部轨迹的目标点
    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
      planner_manager_->global_data_.last_progress_time_ = planner_manager_->global_data_.global_duration_;
    }
    // 如果全局终点到局部轨迹的目标点的距离 < 无人机以做大速度和加速度可飞行的距离，
    // 则局部轨迹的目标点速度设为0
    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    // 否则局部轨迹的目标点速度为全局轨迹在该点的速度
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
    }
  }

} // namespace ego_planner
