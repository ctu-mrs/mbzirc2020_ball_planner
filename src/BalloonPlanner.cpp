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

    switch (m_state)
    {
      case state_enum::waiting_for_detection:
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "[STATEMACH]: Current state: 'WAITING_FOR_DETECTION'");
        /*  //{ */
        
        ROS_INFO_STREAM_THROTTLE(1.0, "[WAITING_FOR_DETECTION]: Going to start point [" << m_start_position.transpose() << "]");
        traj_t result_traj;
        result_traj.header.frame_id = m_world_frame_id;
        result_traj.header.stamp = ros::Time::now();
        result_traj.use_yaw = true;
        result_traj.fly_now = true;
        add_point_to_trajectory(m_start_position, result_traj);
        m_pub_cmd_traj.publish(result_traj);
        
        const auto time_since_last_pred_msg = ros::Time::now() - m_sh_ball_detection->last_message_time();
        if (m_sh_ball_detection->has_data() && time_since_last_pred_msg < ros::Duration(m_max_unseen_time))
        {
          ROS_WARN_STREAM("[WAITING_FOR_DETECTION]: Saw the ball, continuing!");
          m_state = state_enum::following_detection;
        }
        
        //}
      }
      break;
      case state_enum::following_detection:
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "[STATEMACH]: Current state: 'FOLLOWING_DETECTION'");
        /*  //{ */

        auto cur_pos_opt = get_current_position();
        if (m_sh_ball_detection->new_data() && cur_pos_opt.has_value())
        {
          const vec3_t cur_pos = cur_pos_opt.value();
          const auto ball_filtered = *(m_sh_ball_detection->get_data());
          const auto cur_stamp = ros::Time::now();
          const vec3_t ball_pos(ball_filtered.pose.pose.position.x, ball_filtered.pose.pose.position.y, ball_filtered.pose.pose.position.z);
          const vec3_t dir_vec = (ball_pos - cur_pos).normalized();
          const vec3_t offset_vec = m_target_offset*calc_horizontal_offset_vector(dir_vec);
          const double yaw = std::atan2(offset_vec.y(), offset_vec.x());
          const vec3_t tgt_pos = ball_pos - offset_vec;

          auto [follow_traj, follow_traj_duration] =
              sample_trajectory_between_pts(cur_pos, tgt_pos, m_approach_speed, m_trajectory_sampling_dt, yaw);
          ROS_INFO_STREAM_THROTTLE(1.0, "[FOLLOWING_DETECTION]: Follow trajectory: " << follow_traj_duration.toSec() << "s, " << follow_traj.points.size() << "pts");

          follow_traj.header.frame_id = m_world_frame_id;
          follow_traj.header.stamp = cur_stamp;
          follow_traj.use_yaw = true;
          follow_traj.fly_now = true;

          m_pub_cmd_traj.publish(follow_traj);
          if (m_pub_dbg_traj.getNumSubscribers() > 0)
            m_pub_dbg_traj.publish(traj_to_path(follow_traj, m_trajectory_sampling_dt));
          m_prev_plan_stamp = cur_stamp;
        }

        /* check time since last detection message and abort if it's been too long //{ */
        
        {
          const auto time_since_last_pred_msg = ros::Time::now() - m_sh_ball_detection->last_message_time();
          if (time_since_last_pred_msg > ros::Duration(m_max_unseen_time))
          {
            ROS_WARN_STREAM("[FOLLOWING_DETECTION]: Lost ball, (time since last message: " << time_since_last_pred_msg.toSec() << "s). Going back to start!");
            m_state = state_enum::waiting_for_detection;
          }
        }
        
        //}

        if (m_sh_ball_prediction->new_data())
        {
          ROS_WARN_STREAM("[FOLLOWING_DETECTION]: Got prediction, changing state to chasing prediction!");
          m_state = state_enum::chasing_prediction;
        }

        //}
      }
      break;
      case state_enum::chasing_prediction:
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "[STATEMACH]: Current state: 'CHASING_PREDICTION'");
        /*  //{ */

        auto cur_pos_opt = get_current_position();
        if (m_sh_ball_prediction->new_data() && cur_pos_opt.has_value())
        {
          const vec3_t cur_pos = cur_pos_opt.value();
          const auto ball_prediction = *(m_sh_ball_prediction->get_data());
          const auto cur_stamp = ros::Time::now();
          const auto& pred_path = ball_prediction.predicted_path;
          if (pred_path.poses.empty())
          {
            ROS_WARN_STREAM("[CHASING_PREDICTION]: Got empty prediction, cannot do much :(");
          }
          else
          {
            // if the flow gets here, then ball_prediction.predicted_path.poses has at least one element
            const auto ball_pos_ros = pred_path.poses.front();
            const vec3_t ball_pos(ball_pos_ros.pose.position.x, ball_pos_ros.pose.position.y, ball_pos_ros.pose.position.z);
            const vec3_t dir_vec = (ball_pos - cur_pos).normalized();
            const vec3_t offset_vec = m_target_offset*calc_horizontal_offset_vector(dir_vec);

            const auto tgt_path = offset_path(pred_path, offset_vec);
            auto [chase_traj, chase_traj_duration] =
                sample_trajectory_from_path(ros::Time::now(), tgt_path, m_trajectory_sampling_dt, m_max_pts);
            ROS_INFO_STREAM_THROTTLE(1.0, "[CHASING_PREDICTION]: Chase trajectory: " << chase_traj_duration.toSec() << "s, " << chase_traj.points.size() << "pts");
            /* chase_traj = orient_trajectory_yaw(chase_traj, pred_path); */
            for (auto& pt : chase_traj.points)
            {
              const vec3_t cur_traj_pt(pt.x, pt.y, pt.z);
              const vec3_t offset_vec = ball_pos - cur_traj_pt;
              pt.yaw = std::atan2(offset_vec.y(), offset_vec.x());
            }

            chase_traj.header.frame_id = m_world_frame_id;
            chase_traj.header.stamp = cur_stamp;
            chase_traj.use_yaw = true;
            chase_traj.fly_now = true;

            m_pub_cmd_traj.publish(chase_traj);
            if (m_pub_dbg_traj.getNumSubscribers() > 0)
              m_pub_dbg_traj.publish(traj_to_path(chase_traj, m_trajectory_sampling_dt));
            m_prev_plan_stamp = cur_stamp;
          }
        }

        /* check time since last prediction message and abort if it's been too long //{ */
        
        {
          const auto time_since_last_pred_msg = ros::Time::now() - m_sh_ball_detection->last_message_time();
          if (time_since_last_pred_msg > ros::Duration(m_max_unseen_time))
          {
            ROS_WARN_STREAM("[CHASING_PREDICTION]: Lost ball, (time since last message: " << time_since_last_pred_msg.toSec() << "s). Going back to start!");
            m_state = state_enum::waiting_for_detection;
          }
        }
        
        //}

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
    m_target_offset = cfg.trajectory__target_offset;
  }
  //}

  /* calc_offset_vector() method //{ */
  vec3_t BalloonPlanner::calc_horizontal_offset_vector(const vec3_t& dir_vec, const double tolerance)
  {
    vec3_t ret(dir_vec.x(), dir_vec.y(), 0.0);
    if (ret.norm() < tolerance)
      ret = vec3_t(-1.0, 0.0, 0.0);  // if dir_vec is vertical, offset in x by default
    else
      ret = ret.normalized();
    return ret;
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
    return get_transform(frame_id, m_world_frame_id, stamp);
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

  /* offset_path() method //{ */
  path_t BalloonPlanner::offset_path(const path_t& path, const vec3_t& off_vec)
  {
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
                                                                                  const double dt, const double yaw)
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
      const vec4_t cur_pt_with_yaw(cur_pt.x(), cur_pt.y(), cur_pt.z(), yaw);
      add_point_to_trajectory(cur_pt_with_yaw, ret);
      /* add_point_to_path(cur_pt, cur_stamp, ret); */
    }

    ret.header.frame_id = m_world_frame_id;
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

    ret.header.frame_id = m_world_frame_id;
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

  /* limit_cmd_vec_speed() method //{ */
  vec3_t BalloonPlanner::limit_cmd_vec_speed(const vec3_t& cmd_vector, const vec3_t& max_speed, const double dt, size_t max_pts)
  {
    assert(dt > 0.0);
    const vec3_t max_traj_dvec = max_speed*dt;
    vec3_t cmd_vector_speed_limited = cmd_vector;
    double most_limiting = max_pts;
    for (int it = 0; it < vec3_t::RowsAtCompileTime; it++)
    {
      if (max_speed(it) == 0.0)
        cmd_vector_speed_limited(it) = 0.0;
      else if (max_speed(it) > 0.0)
      {
        const double cur_n_pts = std::abs(cmd_vector(it) / max_traj_dvec(it));
        if (cur_n_pts > most_limiting)
          most_limiting = cur_n_pts;
      }
    }
    const double limit_coeff = std::min(most_limiting, double(max_pts)) / double(most_limiting);
    cmd_vector_speed_limited *= limit_coeff;
    return cmd_vector_speed_limited;
  }
  //}

  /* orient_trajectory_yaw() method //{ */
  traj_t BalloonPlanner::orient_trajectory_yaw(const traj_t& traj, const path_t& to_path)
  {
    traj_t ret = traj;
    ret.points.clear();
    const ros::Duration dt_dur(m_trajectory_sampling_dt);
    ros::Time cur_stamp = traj.header.stamp;

    int prev_pose_it = 0;
    for (size_t it = 0; it < traj.points.size(); it++)
    {
      geometry_msgs::PoseStamped prev_pose = to_path.poses.at(prev_pose_it);
      geometry_msgs::PoseStamped next_pose = to_path.poses.at(prev_pose_it + 1);
      // Find prev_pose which is before start_stamp with cur_pose being after start_stamp and next after prev_pose
      while (next_pose.header.stamp < cur_stamp)
      {
        if (prev_pose_it >= (int)to_path.poses.size() - 2)
        {
          ROS_WARN_STREAM_THROTTLE(1.0, "[BalloonPlanner]: Newest path point is older than current time, consider increasing the sampling horizon.");
          break;
        }

        prev_pose_it++;
        prev_pose = to_path.poses.at(prev_pose_it);

        const auto next_pose_it = prev_pose_it + 1;
        next_pose = to_path.poses.at(next_pose_it);
      }

      auto cur_traj_pt_ros = traj.points.at(it);
      const vec3_t cur_traj_pt(cur_traj_pt_ros.x, cur_traj_pt_ros.y, cur_traj_pt_ros.z);
      const vec3_t cur_path_pt = linear_interpolation(cur_stamp, prev_pose, next_pose);
      const vec3_t offset_vec = cur_path_pt - cur_traj_pt;
      const double yaw = std::atan2(offset_vec.y(), offset_vec.x());
      cur_traj_pt_ros.yaw = yaw;
      ret.points.push_back(cur_traj_pt_ros);
      cur_stamp += dt_dur;
    }
    return ret;
  }
  //}

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

    pl.load_param("world_frame_id", m_world_frame_id);
    pl.load_param("uav_frame_id", m_uav_frame_id);
    pl.load_param("approach_speed", m_approach_speed);
    pl.load_param("max_unseen_time", m_max_unseen_time);

    pl.load_param("trajectory/sampling_dt", m_trajectory_sampling_dt);
    pl.load_param("trajectory/horizon", m_trajectory_horizon);
    pl.load_param("trajectory/target_offset", m_target_offset);

    pl.load_param("observing/max_ball_angle", m_observing_max_ball_angle);
    pl.load_param("observing/ball_distance", m_observing_ball_distance);
    pl.load_param("observing/max_forward_speed", m_observing_max_forward_speed);

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
    m_sh_ball_detection = smgr.create_handler<geometry_msgs::PoseWithCovarianceStamped, time_consistent>("ball_filtered", ros::Duration(5.0));
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
    m_pub_dbg_observe_cone = nh.advertise<sensor_msgs::Range>("debig_observe_cone", 1);

    //}

    /* profiler //{ */

    m_profiler_ptr = std::make_unique<mrs_lib::Profiler>(nh, m_node_name, false);

    //}

    /* timers  //{ */

    m_main_loop_timer = nh.createTimer(ros::Duration(planning_period), &BalloonPlanner::main_loop, this);

    //}

    m_max_pts = std::floor(m_trajectory_horizon / m_trajectory_sampling_dt);
    m_state = state_enum::waiting_for_detection;

    ROS_INFO("[%s]: initialized", m_node_name.c_str());
  }

  //}

}  // namespace balloon_planner

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(balloon_planner::BalloonPlanner, nodelet::Nodelet)
