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

    bool lkf_valid = false;
    bool ukf_valid = false;
    if (m_sh_ball_prediction->has_data())
    {
      lkf_valid = m_sh_ball_prediction->peek_data()->filter_state.lkf_state.valid;
      ukf_valid = m_sh_ball_prediction->peek_data()->filter_state.ukf_state.valid;
    }

    const auto cur_pos_yaw_opt = get_current_position();
    if (m_sh_ball_detection->new_data() && cur_pos_yaw_opt.has_value())
    {
      const auto ball_filtered = *(m_sh_ball_detection->peek_data());
      const vec3_t ball_pos(ball_filtered.detection.pose.position.x, ball_filtered.detection.pose.position.y, ball_filtered.detection.pose.position.z);
      const vec4_t& cur_pos_yaw = cur_pos_yaw_opt.value();
      const vec3_t& cur_pos = cur_pos_yaw.block<3, 1>(0, 0);
      const double ball_dist = (cur_pos - ball_pos).norm();
      ROS_INFO_STREAM_THROTTLE(0.5, "[BallPlanner]: Distance from latest ball detection is " << ball_dist << "m.");
    }

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
        
        const auto time_since_last_det_msg = ros::Time::now() - m_sh_ball_detection->last_message_time();
        if (m_sh_ball_detection->has_data() && time_since_last_det_msg < ros::Duration(m_max_unseen_time))
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

        const auto cur_cmd_pos_yaw_opt = get_current_cmd_position();
        if (m_sh_ball_detection->new_data() && cur_cmd_pos_yaw_opt.has_value())
        {
          const auto ball_filtered = *(m_sh_ball_detection->get_data());
          const auto ball_image_pos_opt = calc_ball_image_pos(ball_filtered);
          if (ball_image_pos_opt.has_value())
          {
            const vec4_t& cur_cmd_pos_yaw = cur_cmd_pos_yaw_opt.value();
            const vec3_t& cur_cmd_pos = cur_cmd_pos_yaw.block<3, 1>(0, 0);
            const auto cur_stamp = ros::Time::now();
            const vec3_t ball_pos(ball_filtered.detection.pose.position.x, ball_filtered.detection.pose.position.y, ball_filtered.detection.pose.position.z);
            const vec3_t dir_vec = (ball_pos - cur_cmd_pos).normalized();
            const double yaw = std::atan2(dir_vec.y(), dir_vec.x());
            const vec3_t offset_vec = m_target_offset*calc_horizontal_offset_vector(dir_vec);
            const vec3_t tgt_pos = ball_pos + offset_vec;
  
            auto follow_traj = sample_trajectory_between_pts(cur_cmd_pos, tgt_pos, m_approach_speed, m_trajectory_sampling_dt, yaw);
            const auto follow_traj_duration = trajectory_duration(follow_traj.points.size(), m_trajectory_sampling_dt);
            ROS_INFO_STREAM_THROTTLE(1.0, "[FOLLOWING_DETECTION]: Follow trajectory: " << follow_traj_duration.toSec() << "s, " << follow_traj.points.size() << "pts");
  
            m_pub_cmd_traj.publish(follow_traj);
            if (m_pub_dbg_traj.getNumSubscribers() > 0)
              m_pub_dbg_traj.publish(traj_to_path(follow_traj, m_trajectory_sampling_dt));
            m_prev_plan_stamp = cur_stamp;
          }
        }

        /* /1* check if some prediction is available and if so, change the state //{ *1/ */
        
        /* if (lkf_valid || ukf_valid) */
        /* { */
        /*   ROS_WARN_STREAM("[FOLLOWING_DETECTION]: Got a prediction, changing state to following prediction!"); */
        /*   m_state = state_enum::following_prediction; */
        /* } */
        
        /* //} */

        /* check time since last detection message and abort if it's been too long //{ */
        
        {
          const auto time_since_last_pred_msg = ros::Time::now() - m_sh_ball_detection->last_message_time();
          if (m_sh_ball_detection->has_data() && time_since_last_pred_msg > ros::Duration(m_max_unseen_time))
          {
            ROS_WARN_STREAM("[FOLLOWING_DETECTION]: Lost ball. Going back to start!");
            m_state = state_enum::waiting_for_detection;
          }
        }
        
        //}

        //}
      }
      break;
      case state_enum::following_prediction:
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "[STATEMACH]: Current state: 'FOLLOWING_PREDICTION'");
        /*  //{ */

        const auto cur_cmd_pos_yaw_opt = get_current_cmd_position();
        if (m_sh_ball_prediction->new_data() && cur_cmd_pos_yaw_opt.has_value())
        {
          const vec3_t cur_cmd_pos = cur_cmd_pos_yaw_opt.value().block<3, 1>(0, 0);
          const auto cur_time = ros::Time::now();
          const auto ball_prediction = *(m_sh_ball_prediction->get_data());
          const auto& pred_path = ball_prediction.predicted_path;
          if (pred_path.poses.empty())
          {
            ROS_WARN_STREAM("[FOLLOWING_PREDICTION]: Got empty prediction, cannot do much :(");
          }
          else
          {
            // if the flow gets here, then ball_prediction.predicted_path.poses has at least one element
            const auto ball_pos_ros = pred_path.poses.front().pose.position;
            const vec3_t ball_pos(ball_pos_ros.x, ball_pos_ros.y, ball_pos_ros.z);
            const double ball_expected_speed = ball_prediction.filter_state.expected_speed;

            auto follow_traj_pre = sample_trajectory_between_pts(cur_cmd_pos, ball_pos, m_approach_speed, m_trajectory_sampling_dt);
            const auto follow_traj_pre_duration = trajectory_duration(follow_traj_pre.points.size(), m_trajectory_sampling_dt);
            ROS_INFO_STREAM_THROTTLE(1.0, "[FOLLOWING_PREDICTION]: Approach trajectory: " << follow_traj_pre_duration.toSec() << "s, " << follow_traj_pre.points.size() << "pts");
            follow_traj_pre = orient_trajectory_yaw_observe(follow_traj_pre, pred_path);

            const int n_pts = m_max_pts - follow_traj_pre.points.size();
            auto follow_traj_post = sample_trajectory_from_path(pred_path, m_trajectory_sampling_dt, ball_expected_speed, n_pts);
            const auto follow_traj_post_duration = trajectory_duration(follow_traj_post.points.size(), m_trajectory_sampling_dt);
            ROS_INFO_STREAM_THROTTLE(1.0, "[FOLLOWING_PREDICTION]: Chase trajectory: " << follow_traj_post_duration.toSec() << "s, " << follow_traj_post.points.size() << "pts");
            follow_traj_post = orient_trajectory_yaw_speed(follow_traj_post, pred_path);

            traj_t total_traj = join_trajectories(follow_traj_pre, follow_traj_post);
            total_traj.header.frame_id = m_world_frame_id;
            total_traj.header.stamp = cur_time;
            total_traj.use_yaw = true;
            total_traj.fly_now = true;

            m_pub_cmd_traj.publish(total_traj);
            if (m_pub_dbg_traj.getNumSubscribers() > 0)
              m_pub_dbg_traj.publish(traj_to_path(total_traj, m_trajectory_sampling_dt));
            m_prev_plan_stamp = cur_time;
          }

        }

        /* check if UKF (better) prediction is available and if so, change the state //{ */
        
        auto cur_main_pos_yaw_opt = get_current_position();
        if (ukf_valid && m_sh_ball_detection->new_data() && cur_main_pos_yaw_opt.has_value())
        {
          const auto ball_pos_ros = m_sh_ball_detection->get_data()->detection.pose.position;
          const vec3_t ball_pos(ball_pos_ros.x, ball_pos_ros.y, ball_pos_ros.z);
          const vec3_t cur_main_pos = cur_main_pos_yaw_opt.value().block<3, 1>(0, 0);

          const double ball_det_dist = (cur_main_pos - ball_pos).norm();
          if (ball_det_dist < m_catch_trigger_distance)
          {
            ROS_WARN_STREAM("[FOLLOWING_PREDICTION]: Ball is close and got UKF prediction, changing state to chasing prediction!");
            m_state = state_enum::chasing_prediction;
          }
        }
        
        //}

        /* check time since last prediction message and abort if it's been too long //{ */
        
        if (!lkf_valid && !ukf_valid)
        {
          ROS_WARN_STREAM("[FOLLOWING_PREDICTION]: Lost ball. Going back to start!");
          m_state = state_enum::waiting_for_detection;
        }
        
        //}

        //}
      }
      break;
      case state_enum::chasing_prediction:
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "[STATEMACH]: Current state: 'CHASING_PREDICTION'");
        /*  //{ */

        if (m_sh_ball_prediction->new_data())
        {
          const auto ball_prediction = *(m_sh_ball_prediction->get_data());
          const auto cur_time = ros::Time::now();
          const auto& pred_path = ball_prediction.predicted_path;
          if (pred_path.poses.empty())
          {
            ROS_WARN_STREAM("[CHASING_PREDICTION]: Got empty prediction, cannot do much :(");
          }
          else
          {
            // if the flow gets here, then ball_prediction.predicted_path.poses has at least one element
            const auto ball_pos_ros = pred_path.poses.front().pose.position;
            const vec3_t ball_pos(ball_pos_ros.x, ball_pos_ros.y, ball_pos_ros.z);
            const double ball_expected_speed = ball_prediction.filter_state.expected_speed;

            const int n_pts = m_max_pts;
            auto chase_traj = sample_trajectory_from_path(pred_path, m_trajectory_sampling_dt, ball_expected_speed + m_chase_speed, n_pts);
            const auto chase_traj_duration = trajectory_duration(chase_traj.points.size(), m_trajectory_sampling_dt);
            ROS_INFO_STREAM_THROTTLE(1.0, "[CHASING_PREDICTION]: Chase trajectory: " << chase_traj_duration.toSec() << "s, " << chase_traj.points.size() << "pts");
            chase_traj = orient_trajectory_yaw_speed(chase_traj, pred_path);

            chase_traj.header.frame_id = m_world_frame_id;
            chase_traj.header.stamp = cur_time;
            chase_traj.use_yaw = true;
            chase_traj.fly_now = true;

            m_pub_cmd_traj.publish(chase_traj);
            if (m_pub_dbg_traj.getNumSubscribers() > 0)
              m_pub_dbg_traj.publish(traj_to_path(chase_traj, m_trajectory_sampling_dt));
            m_prev_plan_stamp = cur_time;
          }
        }

        /* check time since last prediction message and abort if it's been too long //{ */
        
        if (!lkf_valid && !ukf_valid)
        {
          ROS_WARN_STREAM("[CHASING_PREDICTION]: Lost ball. Going back to start!");
          m_state = state_enum::waiting_for_detection;
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

  /* yaw_from_quat() method //{ */
  double yaw_from_quat(const quat_t& quat)
  {
    const vec3_t unit_x_rotated = quat*vec3_t::UnitX();
    const double yaw = std::atan2(unit_x_rotated.y(), unit_x_rotated.x());
    return yaw;
  }
  //}

  /* to_eigen() method //{ */
  quat_t to_eigen(const geometry_msgs::Quaternion& quat)
  {
    quat_t ret;
    ret.w() = quat.w;
    ret.x() = quat.x;
    ret.y() = quat.y;
    ret.z() = quat.z;
    return ret;
  }
  //}

  /* to_eigen() method //{ */
  vec3_t to_eigen(const geometry_msgs::Point& point)
  {
    vec3_t ret;
    ret.x() = point.x;
    ret.y() = point.y;
    ret.z() = point.z;
    return ret;
  }
  //}

  /* calc_ball_image_pos() method //{ */
  std::optional<vec3_t> BalloonPlanner::calc_ball_image_pos(const balloon_filter::BallLocation& ball_filtered)
  {
    /* static image_geometry::PinholeCameraModel camera_model; */
    /* camera_model.fromCameraInfo(ball_filtered.camera_info); */
  
    const auto tf_opt = get_transform(ball_filtered.header.frame_id, ball_filtered.camera_info.header.frame_id, ball_filtered.header.stamp);
    if (!tf_opt.has_value())
      return std::nullopt;
    const vec3_t pt3d_det(ball_filtered.detection.pose.position.x, ball_filtered.detection.pose.position.y, ball_filtered.detection.pose.position.z);
    const vec3_t pt3d_tfd = tf_opt.value()*pt3d_det;
    /* geometry_msgs::Point ball_in_camera_frame; */
    /* tf2::doTransform(ball_filtered.detection.pose.position, ball_in_camera_frame, tf_opt.value()); */
  
    /* const cv::Point3d cv_pt3d(ball_in_camera_frame.x, ball_in_camera_frame.y, ball_in_camera_frame.z); */
    /* const cv::Point2d cv_pt2d = camera_model.project3dToPixel(cv_pt3d); */

    /* const double im_width = ball_filtered.camera_info.width; */
    /* const double im_height = ball_filtered.camera_info.height; */
    // convert to range from -1 to 1, where -1 is left border, 0 is center and 1 is right border of the image (similarly for height)
    /* const vec2_t pt2d(2.0*cv_pt2d.x/im_width-1.0, 2.0*cv_pt2d.y/im_height-1.0); */
    /* return pt2d; */
    /* const vec3_t pt3d(ball_in_camera_frame.x, ball_in_camera_frame.y, ball_in_camera_frame.z); */
    return pt3d_tfd;
  }
  //}

  /* pid() method //{ */
  double BalloonPlanner::pid(const double error, const ros::Time& stamp)
  {
    static double prev_error = error;
    static double error_I = 0.0;
    static ros::Time prev_stamp = stamp;
    const ros::Duration dt_dur = stamp - prev_stamp;
    const double dt = dt_dur.toSec();
    if (dt_dur > m_pid_reset_duration)
    {
      prev_error = error;
      error_I = 0.0;
    }
  
    const double error_D = dt == 0.0 ? 0.0 : (error - prev_error)/dt;
    const double result = m_pid_kP*error + m_pid_kI*error_I + m_pid_kD*error_D;
    ROS_INFO("[PID]: P: %.2f,\tI: %.2f,\tD: %.2f,\ty: %.2f", error, error_I, error_D, result);
  
    error_I += dt*error;
    error_I = std::clamp(error_I, -m_pid_max_I, m_pid_max_I);
    prev_error = error;
    return result;
  }
  //}

  /* load_dynparams() method //{ */
  void BalloonPlanner::load_dynparams(drcfg_t cfg)
  {
    m_target_offset = cfg.trajectory__target_offset;
    m_trajectory_horizon = cfg.trajectory__horizon;
    m_max_pts = std::floor(m_trajectory_horizon / m_trajectory_sampling_dt);
    m_approach_speed = cfg.approach_speed;
    m_chase_speed = cfg.chase_speed;
    m_catch_trigger_distance = cfg.catch_trigger_distance;

    m_pid_kP = cfg.pid__kP;
    m_pid_kI = cfg.pid__kI;
    m_pid_kD = cfg.pid__kD;
    m_pid_max_I = cfg.pid__max_I;
    m_pid_reset_duration = ros::Duration(cfg.pid__reset_duration);
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

  /* get_transform_raw() method //{ */
  std::optional<geometry_msgs::TransformStamped> BalloonPlanner::get_transform_raw(const std::string& from_frame_id, const std::string& to_frame_id, ros::Time stamp)
  {
    try
    {
      const ros::Duration timeout(1.0 / 100.0);
      geometry_msgs::TransformStamped transform = m_tf_buffer.lookupTransform(to_frame_id, from_frame_id, stamp, timeout);
      return transform;
    } catch (tf2::TransformException& ex)
    {
      ROS_WARN("Error during transform from \"%s\" frame to \"%s\" frame.\n\tMSG: %s", from_frame_id.c_str(), to_frame_id.c_str(), ex.what());
      return std::nullopt;
    }
  }
  //}

  /* get_transform() method //{ */
  std::optional<Eigen::Affine3d> BalloonPlanner::get_transform(const std::string& from_frame_id, const std::string& to_frame_id, ros::Time stamp)
  {
    const auto tf_opt = get_transform_raw(from_frame_id, to_frame_id, stamp);
    if (tf_opt.has_value())
      return tf2::transformToEigen(tf_opt.value().transform);
    else
      return std::nullopt;
  }
  //}

  /* get_transform_to_world() method //{ */
  std::optional<Eigen::Affine3d> BalloonPlanner::get_transform_to_world(const std::string& frame_id, ros::Time stamp)
  {
    return get_transform(frame_id, m_world_frame_id, stamp);
  }
  //}

  /* get_current_position() method //{ */
  std::optional<vec4_t> BalloonPlanner::get_current_position()
  {
    if (!m_sh_main_odom->has_data())
      return std::nullopt;
    const nav_msgs::Odometry odom = *(m_sh_main_odom->get_data());
    const auto tf_opt = get_transform_to_world(odom.header.frame_id, odom.header.stamp);
    if (!tf_opt.has_value())
      return std::nullopt;
    const vec3_t pos = to_eigen(odom.pose.pose.position);
    const quat_t quat = to_eigen(odom.pose.pose.orientation);
    const vec3_t pos_global = tf_opt.value()*pos;
    const quat_t quat_global = quat_t((tf_opt.value()*quat).rotation());
    const double yaw = yaw_from_quat(quat_global);
    const vec4_t pos_yaw(pos_global.x(), pos_global.y(), pos_global.z(), yaw);
    return pos_yaw;
  }
  //}

  /* get_current_cmd_position() method //{ */
  std::optional<vec4_t> BalloonPlanner::get_current_cmd_position()
  {
    if (!m_sh_cmd_odom->has_data())
      return std::nullopt;
    const nav_msgs::Odometry odom = *(m_sh_cmd_odom->get_data());
    const auto tf_opt = get_transform_to_world(odom.header.frame_id, odom.header.stamp);
    if (!tf_opt.has_value())
      return std::nullopt;
    const vec3_t pos = to_eigen(odom.pose.pose.position);
    const quat_t quat = to_eigen(odom.pose.pose.orientation);
    const vec3_t pos_global = tf_opt.value()*pos;
    const quat_t quat_global = quat_t((tf_opt.value()*quat).rotation());
    const double yaw = yaw_from_quat(quat_global);
    const vec4_t pos_yaw(pos_global.x(), pos_global.y(), pos_global.z(), yaw);
    return pos_yaw;
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

  /* linear_interpolation_quat() method //{ */
  quat_t linear_interpolation_quat(const ros::Time& to_time, const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2)
  {
    const ros::Duration time_diff = pose2.header.stamp - pose1.header.stamp;
    assert(!(time_diff.isZero() && to_time != pose1.header.stamp));

    const quat_t quat1 = to_eigen(pose1.pose.orientation);
    if (time_diff.isZero())
      return quat1;
    const quat_t quat2 = to_eigen(pose2.pose.orientation);
    const ros::Duration desired_time_diff = to_time - pose1.header.stamp;
    const double coeff = desired_time_diff.toSec()/time_diff.toSec();
    const quat_t interp_quat = quat1.slerp(coeff, quat2);
    return interp_quat;
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

  /* trajectory_duration() method //{ */
  ros::Duration BalloonPlanner::trajectory_duration(const int n_traj_pts, const double dt)
  {
    const ros::Duration traj_dur(n_traj_pts > 0 ? (n_traj_pts - 1) * dt : 0);
    return traj_dur;
  }
  //}

  /* sample_trajectory_between_pts() method //{ */
  traj_t BalloonPlanner::sample_trajectory_between_pts(const vec3_t& from_pt, const vec3_t& to_pt, const double speed,
                                                                                  const double dt, const double yaw)
  {
    assert(speed > 0.0);
    assert(dt > 0.0);

    const vec3_t diff_vec = to_pt - from_pt;
    const double dist = diff_vec.norm();
    const size_t n_pts = std::min(size_t(std::ceil(dist / (speed * dt))) + 1, m_max_pts);
    const ros::Duration traj_dur = trajectory_duration(n_pts, dt);
    const double speed_scaled = dist / traj_dur.toSec();
    /* ROS_INFO_STREAM("[]: scaled speed : " << speed_scaled << "m/s (orig: " << speed << "m/s)"); */
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

    return ret;
  }
  //}

  /* sample_trajectory_from_path() method //{ */
  traj_t BalloonPlanner::sample_trajectory_from_path(const path_t& path, const double dt, const double speed, const size_t n_pts)
  {
    assert(!path.poses.empty());
    traj_t ret;
    ret.points.reserve(n_pts);

    double dlen_remaining = dt*speed;
    int prev_pose_it = 0;
    for (size_t it = 0; it < n_pts; it++)
    {
      geometry_msgs::PoseStamped prev_pose = path.poses.at(prev_pose_it);
      geometry_msgs::PoseStamped next_pose = path.poses.at(prev_pose_it + 1);
      vec3_t prev_pos = to_eigen(prev_pose.pose.position);
      vec3_t next_pos = to_eigen(next_pose.pose.position);
      double dlen_cur = (next_pos - prev_pos).norm();
      // Find prev_pose which is before start_stamp with cur_pose being after start_stamp and next after prev_pose
      while (dlen_cur < dlen_remaining)
      {
        if (prev_pose_it >= (int)path.poses.size() - 2)
        {
          ROS_WARN_STREAM_THROTTLE(1.0, "[BalloonPlanner]: Newest path point is older than current time, consider increasing the sampling horizon.");
          break;
        }

        dlen_remaining -= dlen_cur;
        prev_pose_it++;
        prev_pose = path.poses.at(prev_pose_it);

        const auto next_pose_it = prev_pose_it + 1;
        next_pose = path.poses.at(next_pose_it);
        prev_pos = to_eigen(prev_pose.pose.position);
        next_pos = to_eigen(next_pose.pose.position);
        dlen_cur = (next_pos - prev_pos).norm();
      }
      const vec3_t cur_pt = prev_pos + dlen_remaining*(next_pos - prev_pos)/dlen_cur;
      add_point_to_trajectory(cur_pt, ret);
      dlen_remaining += dt*speed;
    }
    ret.header.frame_id = m_world_frame_id;
    ret.header.stamp = ros::Time::now();
    m_pub_dbg_follow_traj.publish(traj_to_path(ret, dt));

    return ret;
  }
  //}

  /* sample_trajectory_from_path() method //{ */
  traj_t BalloonPlanner::sample_trajectory_from_path(const ros::Time& start_stamp, const path_t& path, const double dt, const size_t n_pts)
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
    ret.header.frame_id = m_world_frame_id;
    ret.header.stamp = ros::Time::now();
    m_pub_dbg_follow_traj.publish(traj_to_path(ret, dt));

    return ret;
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
  traj_t BalloonPlanner::orient_trajectory_yaw(const traj_t& traj, const vec3_t& to_point)
  {
    traj_t ret = traj;
    ret.points.clear();
    const ros::Duration dt_dur(m_trajectory_sampling_dt);
    ros::Time cur_stamp = traj.header.stamp;
    for (auto& traj_pt : ret.points)
    {
      const vec3_t cur_traj_pt(traj_pt.x, traj_pt.y, traj_pt.z);
      const vec3_t offset_vec = to_point - cur_traj_pt;
      const double yaw = std::atan2(offset_vec.y(), offset_vec.x());
      traj_pt.yaw = yaw;
      ret.points.push_back(traj_pt);
      cur_stamp += dt_dur;
    }
    return ret;
  }
  //}

  /* orient_trajectory_yaw_observe() method //{ */
  traj_t BalloonPlanner::orient_trajectory_yaw_observe(const traj_t& traj, const path_t& to_path)
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

  /* orient_trajectory_yaw_speed() method //{ */
  traj_t BalloonPlanner::orient_trajectory_yaw_speed(const traj_t& traj, const path_t& to_path)
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
      const quat_t cur_path_quat = linear_interpolation_quat(cur_stamp, prev_pose, next_pose);
      cur_traj_pt_ros.yaw = yaw_from_quat(cur_path_quat);
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

  /* path_to_traj() method //{ */
  traj_t BalloonPlanner::path_to_traj(const path_t& path)
  {
    traj_t ret;
    ret.header = path.header;
    for (const auto& path_pose : path.poses)
    {
      traj_t::_points_type::value_type traj_pt;
      traj_pt.x = path_pose.pose.position.x;
      traj_pt.y = path_pose.pose.position.y;
      traj_pt.z = path_pose.pose.position.z;
      traj_pt.yaw = yaw_from_quat(to_eigen(path_pose.pose.orientation));
      ret.points.push_back(traj_pt);
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
    pl.load_param("max_unseen_time", m_max_unseen_time);

    pl.load_param("trajectory/sampling_dt", m_trajectory_sampling_dt);

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
    m_sh_ball_detection = smgr.create_handler<balloon_filter::BallLocation>("ball_filtered", ros::Duration(5.0));
    m_sh_ball_prediction = smgr.create_handler<balloon_filter::BallPrediction>("ball_prediction", ros::Duration(5.0));
    m_sh_cmd_odom = smgr.create_handler<nav_msgs::Odometry>("cmd_odom", ros::Duration(5.0));
    m_sh_main_odom = smgr.create_handler<nav_msgs::Odometry>("main_odom", ros::Duration(5.0));

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

    m_state = state_enum::waiting_for_detection;

    ROS_INFO("[%s]: initialized", m_node_name.c_str());
  }

  //}

}  // namespace balloon_planner

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(balloon_planner::BalloonPlanner, nodelet::Nodelet)
