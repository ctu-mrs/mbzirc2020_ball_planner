#include <balloon_planner/BalloonPlanner.h>

namespace balloon_planner
{
  /* time_to_fly() method //{ */
  ros::Duration time_to_fly(const vec3_t& from_pt, const vec3_t& to_pt, const double speed)
  {
    const double dist = (from_pt - to_pt).norm();
    ros::Duration dur(dist / speed);
    return dur;
  }
  //}

  /* add_point_to_trajectory() method //{ */
  void add_point_to_trajectory(const vec3_t pt, traj_t& traj)
  {
    mrs_msgs::TrackerPoint tr_pt;
    tr_pt.x = pt.x();
    tr_pt.y = pt.y();
    tr_pt.z = pt.z();
    traj.points.push_back(tr_pt);
  }
  
  void add_point_to_trajectory(const vec4_t pt, traj_t& traj)
  {
    mrs_msgs::TrackerPoint tr_pt;
    tr_pt.x = pt.x();
    tr_pt.y = pt.y();
    tr_pt.z = pt.z();
    tr_pt.yaw = pt.w();
    traj.points.push_back(tr_pt);
  }
  //}

  /* main_loop() method //{ */
  void BalloonPlanner::main_loop([[maybe_unused]] const ros::TimerEvent& evt)
  {
    load_dynparams(m_drmgr_ptr->config);

    if (m_sh_ball_prediction->has_data() && !m_sh_ball_prediction->peek_data()->predicted_path.poses.empty())
      m_last_pred_path_stamp = m_sh_ball_prediction->peek_data()->header.stamp;

    switch (m_state)
    {
      case state_enum::waiting_for_prediction:
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "[STATEMACH]: Current state: 'WAITING_FOR_PREDICTION'");
        /*  //{ */
        
        auto cur_pos_res = get_current_position();
        if (cur_pos_res.has_value() && m_sh_ball_prediction->new_data())
        {
          const auto cur_ball_msg = m_sh_ball_prediction->get_data();
          const auto cur_ball_rpos = cur_ball_msg->filter_state.ukf_state.position;
          const vec3_t cur_ball_pos(cur_ball_rpos.x, cur_ball_rpos.y, cur_ball_rpos.z);
        
          // if no prediction is available, just follow the current ball position
          const auto tf_msg2uav_opt = get_transform(cur_ball_msg->header.frame_id, m_uav_frame_id, cur_ball_msg->header.stamp);
          if (tf_msg2uav_opt.has_value())
          {
            std::optional<Eigen::Affine3d> tf_uav2world_opt;
            // this is usually the case and it will save us a redundant TF query
            if (cur_ball_msg->header.frame_id == m_world_frame)
              tf_uav2world_opt = tf_msg2uav_opt.value().inverse();
            else
              tf_uav2world_opt = get_transform(m_world_frame, m_uav_frame_id, cur_ball_msg->header.stamp);
        
            // All required transformations are known
            if (tf_uav2world_opt.has_value())
            {
              // vector from the MAV to the ball (ball position in MAV frame)
              const vec3_t ball_mav_vec = tf_msg2uav_opt.value()*cur_ball_pos;
              const vec3_t x_vec(1, 0, 0);
              const auto rot = mrs_lib::angleaxis_between(ball_mav_vec, x_vec);
              double cmd_angle = rot.angle();
              if (std::abs(rot.angle()) > m_observing_max_ball_angle)
                cmd_angle = mrs_lib::sign(rot.angle())*m_observing_max_ball_angle;
              const Eigen::AngleAxisd cmd_rot(cmd_angle, rot.axis());

              const vec3_t uav_cmd_vec = m_observing_ball_distance*(cmd_rot*ball_mav_vec.normalized());
              const vec3_t world_cmd_vec = tf_uav2world_opt.value()*uav_cmd_vec;
              auto cmd_traj = point_to_traj(world_cmd_vec, m_max_pts);
              cmd_traj.header.frame_id = m_world_frame;
              cmd_traj.header.stamp = ros::Time::now();
              cmd_traj.fly_now = true;
              cmd_traj.use_yaw = false;
              m_pub_cmd_traj.publish(cmd_traj);
              ROS_WARN_STREAM_THROTTLE(1.0, "[WAITING_FOR_PREDICTION]: Going to point [" << world_cmd_vec.transpose() << "] (ball angle is " << rot.angle() << "rad)");
            }
          }
        
          // if prediction is available, switch the state to start following the predicted path
          if (!cur_ball_msg->predicted_path.poses.empty())
          {
            m_path_offset = (cur_ball_pos - cur_pos_res.value()).norm();
            m_prev_plan_stamp = ros::Time::now();
            m_state = state_enum::following;
          }
        }
        
        //}
      }
      break;
      case state_enum::following:
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "[STATEMACH]: Current state: 'FOLLOWING'");
        /*  //{ */

        auto cur_pos_res = get_current_position();
        if (m_sh_ball_prediction->new_data() && cur_pos_res.has_value())
        {
          const vec3_t cur_pos = cur_pos_res.value();
          const auto ball_prediction = *(m_sh_ball_prediction->get_data());
          const auto& pred_path = ball_prediction.predicted_path;
          const auto& plane_params = ball_prediction.filter_state.fitted_plane;
          /* const auto& ukf_state = ball_prediction.filter_state.ukf_state; */
          /* const auto& est_speed = ukf_state.speed; */
          const auto cur_stamp = ros::Time::now();
          if (pred_path.poses.empty())
          {
            ROS_ERROR_STREAM_THROTTLE(1.0, "[BalloonPlanner]: Received empty predicted path, cannot plan trajectory!");
            return;
          }
          if (pred_path.poses.back().header.stamp < cur_stamp)
          {
            ROS_ERROR_STREAM_THROTTLE(1.0, "[BalloonPlanner]: Newest predicted pose is older that current time, cannot plan trajectory!");
            return;
          }

          const auto offset_vector = calc_path_offset_vector(plane_params, cur_pos);
          const auto tgt_path = offset_path(pred_path, offset_vector, m_path_offset);
          m_path_offset -= m_approach_speed * (cur_stamp - m_prev_plan_stamp).toSec();

          const auto follow_traj_start_time = ros::Time::now();
          const auto [follow_traj, follow_traj_duration] =
              sample_trajectory_from_path(follow_traj_start_time, tgt_path, m_trajectory_sampling_dt, m_max_pts);
          ROS_INFO_STREAM_THROTTLE(1.0, "[BalloonPlanner]: Follow trajectory: " << follow_traj_duration.toSec() << "s, " << follow_traj.points.size() << "pts");

          traj_t result_traj = orient_trajectory_yaw(follow_traj, pred_path);
          result_traj.header.frame_id = m_world_frame;
          result_traj.header.stamp = cur_stamp;
          result_traj.use_yaw = true;
          result_traj.fly_now = true;
          assert(result_traj.points.size() == m_max_pts);

          m_pub_cmd_traj.publish(result_traj);
          if (m_pub_dbg_traj.getNumSubscribers() > 0)
            m_pub_dbg_traj.publish(traj_to_path(result_traj, m_trajectory_sampling_dt));
          m_prev_plan_stamp = cur_stamp;
        }

        {
          /* const auto last_message_time = m_sh_ball_prediction->last_message_time(); */
          const auto time_since_last_path_prediction = ros::Time::now() - m_last_pred_path_stamp;
          if (time_since_last_path_prediction > ros::Duration(m_max_unseen_time))
          {
            m_state = state_enum::going_back;
          }
        }

        //}
      }
      break;
      case state_enum::going_back:
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "[STATEMACH]: Current state: 'GOING_BACK'");
        /*  //{ */
        
        ROS_INFO_STREAM_THROTTLE(1.0, "[GOING_BACK]: Going to start point [" << m_start_position.transpose() << "]");
        traj_t result_traj;
        result_traj.header.frame_id = m_world_frame;
        result_traj.header.stamp = ros::Time::now();
        result_traj.use_yaw = true;
        result_traj.fly_now = true;
        add_point_to_trajectory(m_start_position, result_traj);
        m_pub_cmd_traj.publish(result_traj);
        m_state = state_enum::waiting_for_prediction;
        
        //}
      }
      break;
    }
  }
  //}

  // --------------------------------------------------------------
  // |                       Helper methods                       |
  // --------------------------------------------------------------

  /* load_dynparams() method //{ */
  void BalloonPlanner::load_dynparams(drcfg_t cfg)
  {
  }
  //}

  /* get_transform() method //{ */
  std::optional<Eigen::Affine3d> BalloonPlanner::get_transform(const std::string& from_frame_id, const std::string& to_frame_id, ros::Time stamp)
  {
    try
    {
      const ros::Duration timeout(1.0 / 100.0);
      geometry_msgs::TransformStamped transform = m_tf_buffer.lookupTransform(to_frame_id, from_frame_id, stamp, timeout);
      return tf2::transformToEigen(transform.transform);
    } catch (tf2::TransformException& ex)
    {
      ROS_WARN("Error during transform from \"%s\" frame to \"%s\" frame.\n\tMSG: %s", from_frame_id.c_str(), to_frame_id.c_str(), ex.what());
      return std::nullopt;
    }
  }
  //}

  /* get_transform_to_world() method //{ */
  std::optional<Eigen::Affine3d> BalloonPlanner::get_transform_to_world(const std::string& frame_id, ros::Time stamp)
  {
    return get_transform(frame_id, m_world_frame, stamp);
  }
  //}

  /* get_current_position() method //{ */
  std::optional<vec3_t> BalloonPlanner::get_current_position()
  {
    const auto uav2world_tf_opt = get_transform_to_world(m_uav_frame_id, ros::Time(0.0));
    if (uav2world_tf_opt.has_value())
      return uav2world_tf_opt.value() * vec3_t(0, 0, 0);
    else
      return std::nullopt;
  }
  //}

  /* sign() method //{ */
  template <typename T>
  int sign(T val)
  {
    return (T(0) < val) - (val < T(0));
  }
  //}

  /* calc_path_offset_vector() method //{ */
  vec3_t BalloonPlanner::calc_path_offset_vector(const plane_t& plane_params, const vec3_t& towards_pt, const double tolerance)
  {
    const Eigen::Vector4d plane_theta(plane_params.normal.x, plane_params.normal.y, plane_params.normal.z, plane_params.offset);
    const Eigen::Vector4d pt_tmp(towards_pt.x(), towards_pt.y(), towards_pt.z(), 1.0);
    const double orientation = plane_theta.transpose() * pt_tmp;
    const vec3_t normal = sign(orientation) * vec3_t(plane_params.normal.x, plane_params.normal.y, plane_params.normal.z);
    vec3_t ret(normal.x(), normal.y(), 0.0);
    if (ret.norm() < tolerance)
      ret = vec3_t(-1.0, 0.0, 0.0);  // if the plane is horizontal, offset in x by default
    else
      ret = ret.normalized();
    return ret;
  }
  //}

  /* offset_path() method //{ */
  path_t BalloonPlanner::offset_path(const path_t& path, const vec3_t& vector, const double offset)
  {
    const vec3_t off_vec = vector * offset;
    path_t ret = path;
    for (auto& pose : ret.poses)
    {
      pose.pose.position.x += off_vec.x();
      pose.pose.position.y += off_vec.y();
      pose.pose.position.z += off_vec.z();
    }
    return ret;
  }
  //}

  /* find_closest_line_param() method //{ */
  double find_closest_line_param(const vec3_t& to_pt, const vec3_t& line_pt1, const vec3_t& line_pt2)
  {
    const vec3_t line_dir = line_pt2 - line_pt1;
    const vec3_t diff_vec = to_pt - line_pt1;
    const double param = diff_vec.dot(line_dir);
    return param;
  }
  //}

  /* linear_interpolation() method //{ */
  vec3_t linear_interpolation(const ros::Time& to_time, const geometry_msgs::PoseStamped& line_pt1, const geometry_msgs::PoseStamped& line_pt2)
  {
    const ros::Duration time_diff = line_pt2.header.stamp - line_pt1.header.stamp;
    assert(!(time_diff.isZero() && to_time != line_pt1.header.stamp));

    const vec3_t pt1(line_pt1.pose.position.x, line_pt1.pose.position.y, line_pt1.pose.position.z);
    if (time_diff.isZero())
      return pt1;
    const vec3_t pt2(line_pt2.pose.position.x, line_pt2.pose.position.y, line_pt2.pose.position.z);
    const vec3_t pt_diff = pt2 - pt1;
    const ros::Duration desired_time_diff = to_time - line_pt1.header.stamp;
    const vec3_t interp_pt = pt1 + pt_diff / time_diff.toSec() * desired_time_diff.toSec();
    return interp_pt;
  }
  //}

  /* find_approach_pt() method //{ */
  vec3_t BalloonPlanner::find_approach_pt(const vec3_t& from_pt, const ros::Time& from_time, const path_t& to_path, const double speed)
  {
    assert(!to_path.poses.empty());
    vec3_t closest_pt;
    ros::Time closest_time;
    bool closest_time_set = false;
    /* double closest_param = std::numeric_limits<double>::infinity(); */
    /* double closest_param_unclamped = std::numeric_limits<double>::infinity(); */

    geometry_msgs::PoseStamped prev_pose;
    bool prev_pose_set = false;
    for (const auto& pose : to_path.poses)
    {
      if (prev_pose_set)
      {
        const vec3_t line_pt1(prev_pose.pose.position.x, prev_pose.pose.position.y, prev_pose.pose.position.z);
        const vec3_t line_pt2(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        const auto cur_param_unclamped = find_closest_line_param(from_pt, line_pt1, line_pt2);
        /* ROS_WARN_STREAM("cur param (unclamped): " << cur_param_unclamped); */
        const auto cur_param = std::clamp(cur_param_unclamped, 0.0, 1.0);
        assert(cur_param >= 0.0);
        assert(cur_param <= 1.0);
        const ros::Time t1 = prev_pose.header.stamp;
        const ros::Time t2 = pose.header.stamp;
        const vec3_t approach_pt = line_pt1 + (line_pt2 - line_pt1) * cur_param;
        const ros::Duration approach_dur = time_to_fly(from_pt, approach_pt, speed);
        const ros::Time approach_time = from_time + approach_dur;

        if (approach_time > t1 && approach_time < t2 && (!closest_time_set || approach_time < closest_time))
        {
          closest_pt = approach_pt;
          closest_time = approach_time;
          closest_time_set = true;
          /* closest_param = cur_param; */
          /* closest_param_unclamped = cur_param_unclamped; */
        }
      }
      prev_pose = pose;
      prev_pose_set = true;
    }

    if (!closest_time_set)
    {
      const auto last_pt = to_path.poses.back();
      closest_pt = vec3_t(last_pt.pose.position.x, last_pt.pose.position.y, last_pt.pose.position.z);
      const ros::Duration approach_dur = time_to_fly(from_pt, closest_pt, speed);
      closest_time = from_time + approach_dur;
    }
    /* ROS_WARN_STREAM("closest param:" << closest_param); */
    /* ROS_WARN_STREAM("closest param (unclamped): " << closest_param_unclamped); */
    /* ROS_WARN_STREAM("ideal approach time: " << closest_time); */

    return closest_pt;
  }
  //}

  /* sample_trajectory_between_pts() method //{ */
  std::tuple<traj_t, ros::Duration> BalloonPlanner::sample_trajectory_between_pts(const vec3_t& from_pt, const vec3_t& to_pt, const double speed,
                                                                                  const double dt)
  {
    assert(speed > 0.0);
    assert(dt > 0.0);

    const vec3_t diff_vec = to_pt - from_pt;
    const double dist = diff_vec.norm();
    const size_t n_pts = std::min(size_t(std::ceil(dist / (speed * dt))) + 1, m_max_pts);
    const ros::Duration traj_dur((n_pts - 1) * dt);
    const double speed_scaled = dist / traj_dur.toSec();
    ROS_INFO_STREAM("[]: scaled speed : " << speed_scaled << "m/s (orig: " << speed << "m/s)");
    const vec3_t d_vec = speed_scaled * dt * diff_vec / dist;

    traj_t ret;
    ret.points.reserve(n_pts);
    for (size_t it = 0; it < n_pts; it++)
    {
      const vec3_t cur_pt = from_pt + it * d_vec;
      add_point_to_trajectory(cur_pt, ret);
      /* add_point_to_path(cur_pt, cur_stamp, ret); */
    }

    ret.header.frame_id = m_world_frame;
    ret.header.stamp = ros::Time::now();
    m_pub_dbg_approach_traj.publish(traj_to_path(ret, dt));

    return {ret, traj_dur};
  }
  //}

  /* sample_trajectory_from_path() method //{ */
  std::tuple<traj_t, ros::Duration> BalloonPlanner::sample_trajectory_from_path(const ros::Time& start_stamp, const path_t& path, const double dt,
                                                                                const size_t n_pts)
  {
    assert(!path.poses.empty());
    /* const size_t n_pts = std::min(m_max_pts, size_t(std::floor(dur.toSec() / dt))); */
    const ros::Duration dt_dur(dt);

    traj_t ret;
    ret.points.reserve(n_pts);

    ros::Time cur_stamp = start_stamp;
    int prev_pose_it = 0;
    for (size_t it = 0; it < n_pts; it++)
    {
      geometry_msgs::PoseStamped prev_pose = path.poses.at(prev_pose_it);
      geometry_msgs::PoseStamped next_pose = path.poses.at(prev_pose_it + 1);
      // Find prev_pose which is before start_stamp with cur_pose being after start_stamp and next after prev_pose
      while (next_pose.header.stamp < cur_stamp)
      {
        if (prev_pose_it >= (int)path.poses.size() - 2)
        {
          ROS_WARN_STREAM_THROTTLE(1.0, "[BalloonPlanner]: Newest path point is older than current time, consider increasing the sampling horizon.");
          break;
        }

        prev_pose_it++;
        prev_pose = path.poses.at(prev_pose_it);

        const auto next_pose_it = prev_pose_it + 1;
        next_pose = path.poses.at(next_pose_it);
      }

      if (it == 0)
      {
        m_pub_dbg_path_pose1.publish(prev_pose);
        m_pub_dbg_path_pose2.publish(next_pose);

        /*       ROS_INFO_STREAM("cur_stamp: " << std::endl << cur_stamp); */
        /*       ROS_INFO_STREAM("prev_pose: " << std::endl << prev_pose); */
        /*       ROS_INFO_STREAM("next_pose: " << std::endl << next_pose); */
      }

      const auto cur_pt = linear_interpolation(cur_stamp, prev_pose, next_pose);
      add_point_to_trajectory(cur_pt, ret);

      cur_stamp += dt_dur;
    }
    const ros::Duration traj_dur((n_pts - 1) * dt);

    ret.header.frame_id = m_world_frame;
    ret.header.stamp = ros::Time::now();
    m_pub_dbg_follow_traj.publish(traj_to_path(ret, dt));

    return {ret, traj_dur};
  }
  //}

  /* join_trajectories() method //{ */
  traj_t BalloonPlanner::join_trajectories(const traj_t& traj1, const traj_t& traj2)
  {
    traj_t ret;
    ret.header = traj1.header;
    ret.points.insert(std::end(ret.points), std::begin(traj1.points), std::end(traj1.points));
    ret.points.insert(std::end(ret.points), std::begin(traj2.points), std::end(traj2.points));
    return ret;
  }
  //}

  traj_t BalloonPlanner::orient_trajectory_yaw(const traj_t& traj, const path_t& to_path)
  {
    return traj;  // TODO
  }

  /* traj_to_path() method //{ */
  path_t BalloonPlanner::traj_to_path(const traj_t& traj, const double traj_dt)
  {
    path_t ret;
    ret.header = traj.header;

    const ros::Duration dt_dur(traj_dt);
    ros::Time cur_stamp = traj.header.stamp;
    for (const auto& tr_pt : traj.points)
    {
      const quat_t quat(Eigen::AngleAxisd(tr_pt.yaw, vec3_t::UnitZ()));

      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = traj.header.frame_id;
      pose.header.stamp = cur_stamp;

      pose.pose.position.x = tr_pt.x;
      pose.pose.position.y = tr_pt.y;
      pose.pose.position.z = tr_pt.z;

      pose.pose.orientation.w = quat.w();
      pose.pose.orientation.x = quat.x();
      pose.pose.orientation.y = quat.y();
      pose.pose.orientation.z = quat.z();

      ret.poses.push_back(pose);

      cur_stamp += dt_dur;
    }
    return ret;
  }
  //}

  /* point_to_traj() method //{ */
  traj_t BalloonPlanner::point_to_traj(const vec3_t& point, const size_t n_pts)
  {
    traj_t ret;
    ret.points.reserve(n_pts);
    mrs_msgs::TrackerPoint tr_point;
    tr_point.x = point.x();
    tr_point.y = point.y();
    tr_point.z = point.z();
    for (size_t it = 0; it < n_pts; it++)
      ret.points.push_back(tr_point);
    return ret;
  }
  //}

  /* onInit() //{ */

  void BalloonPlanner::onInit()
  {

    ROS_INFO("[%s]: Initializing", m_node_name.c_str());
    ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
    ros::Time::waitForValid();

    /* load parameters //{ */

    // LOAD DYNAMIC PARAMETERS
    ROS_INFO("[%s]: LOADING DYNAMIC PARAMETERS", m_node_name.c_str());
    m_drmgr_ptr = std::make_unique<drmgr_t>(nh, m_node_name);

    ROS_INFO("[%s]: LOADING STATIC PARAMETERS", m_node_name.c_str());
    mrs_lib::ParamLoader pl(nh, m_node_name);

    const double planning_period = pl.load_param2<double>("planning_period");

    pl.load_param("world_frame", m_world_frame);
    pl.load_param("uav_frame_id", m_uav_frame_id);
    pl.load_param("approach_speed", m_approach_speed);
    pl.load_param("max_unseen_time", m_max_unseen_time);

    pl.load_param("trajectory/sampling_dt", m_trajectory_sampling_dt);
    pl.load_param("trajectory/horizon", m_trajectory_horizon);

    pl.load_param("observing/max_ball_angle", m_observing_max_ball_angle);
    pl.load_param("observing/ball_distance", m_observing_ball_distance);

    pl.load_matrix_static<4, 1>("start_position", m_start_position);

    if (!pl.loaded_successfully())
    {
      ROS_ERROR("Some compulsory parameters were not loaded successfully, ending the node");
      ros::shutdown();
    }

    if (!m_drmgr_ptr->loaded_successfully())
    {
      ROS_ERROR("Some dynamic parameter default values were not loaded successfully, ending the node");
      ros::shutdown();
    }

    //}

    /* subscribers //{ */

    m_tf_listener_ptr = std::make_unique<tf2_ros::TransformListener>(m_tf_buffer, m_node_name);
    mrs_lib::SubscribeMgr smgr(nh);
    constexpr bool time_consistent = true;
    m_sh_ball_prediction = smgr.create_handler<balloon_filter::BallPrediction, time_consistent>("ball_prediction", ros::Duration(5.0));
    /* m_sh_fitted_plane = smgr.create_handler<balloon_filter::Plane, time_consistent>("ball_fitted_plane", ros::Duration(5.0)); */

    //}

    /* publishers //{ */

    m_pub_cmd_traj = nh.advertise<mrs_msgs::TrackerTrajectory>("commanded_trajectory", 1);
    m_pub_dbg_traj = nh.advertise<nav_msgs::Path>("debug_trajectory", 1);
    m_pub_dbg_approach_traj = nh.advertise<nav_msgs::Path>("debug_approach_trajectory", 1);
    m_pub_dbg_follow_traj = nh.advertise<nav_msgs::Path>("debug_follow_trajectory", 1);
    m_pub_dbg_approach_pt = nh.advertise<geometry_msgs::PointStamped>("debug_approach_point", 1);
    m_pub_dbg_path_pose1 = nh.advertise<geometry_msgs::PoseStamped>("debug_path_pose1", 1);
    m_pub_dbg_path_pose2 = nh.advertise<geometry_msgs::PoseStamped>("debug_path_pose2", 1);

    //}

    /* profiler //{ */

    m_profiler_ptr = std::make_unique<mrs_lib::Profiler>(nh, m_node_name, false);

    //}

    /* timers  //{ */

    m_main_loop_timer = nh.createTimer(ros::Duration(planning_period), &BalloonPlanner::main_loop, this);

    //}

    m_max_pts = std::floor(m_trajectory_horizon / m_trajectory_sampling_dt);
    m_state = state_enum::going_back;

    ROS_INFO("[%s]: initialized", m_node_name.c_str());
  }

  //}

}  // namespace balloon_planner

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(balloon_planner::BalloonPlanner, nodelet::Nodelet)
