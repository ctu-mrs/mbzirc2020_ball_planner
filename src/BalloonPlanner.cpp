#include <balloon_planner/BalloonPlanner.h>

namespace balloon_planner
{

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
    /* set constraints, set last seen yaw etc. according to ball position if available //{ */
    
    if (m_sh_ball_detection->new_data() && cur_pos_yaw_opt.has_value())
    {
      const auto ball_filtered = *(m_sh_ball_detection->peek_data());
      const vec3_t ball_pos(ball_filtered.detection.pose.position.x, ball_filtered.detection.pose.position.y, ball_filtered.detection.pose.position.z);
      const vec4_t& cur_pos_yaw = cur_pos_yaw_opt.value();
      const vec3_t& cur_pos = cur_pos_yaw.block<3, 1>(0, 0);
      const double& cur_yaw = cur_pos_yaw.w();
      const vec3_t ball_dir = cur_pos - ball_pos;

      const double ball_relative_yaw = mrs_lib::normalize_angle(std::atan2(ball_dir.y(), ball_dir.x()) - cur_yaw);
      m_last_seen_relative_yaw = ball_relative_yaw;

      const double ball_dist = ball_dir.norm();
      const auto desired_constraints = pick_constraints(ball_dist);
      ROS_INFO_STREAM_THROTTLE(0.5, "[BallPlanner]: Distance from latest ball detection is " << ball_dist << "m at " << ball_relative_yaw << " relative yaw, setting '" << desired_constraints << "' constraints.");
      set_constraints(desired_constraints);
    }
    
    //}

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
        if (m_sh_ball_detection->has_data() && time_since_last_det_msg < m_max_unseen_dur)
        {
          ROS_WARN_STREAM("[WAITING_FOR_DETECTION]: Saw the ball, continuing!");
          m_following_start = ros::Time::now();
          m_ball_positions.clear();
          m_state = state_enum::following_detection;
        }
        
        //}
      }
      break;
      case state_enum::lost_glancing:
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "[STATEMACH]: Current state: 'LOST_GLANCING'");
        /*  //{ */

        /* the main action //{ */
        
        const auto cur_cmd_pos_yaw_opt = get_current_cmd_position();
        if (m_sh_ball_detection->new_data() && cur_cmd_pos_yaw_opt.has_value())
        {
          vec4_t cur_pos_yaw = cur_cmd_pos_yaw_opt.value();
          traj_t glance_traj;
          for (size_t it = 0; it < m_max_pts; it++)
          {
            cur_pos_yaw.w() += m_trajectory_sampling_dt*m_glancing_yaw_rate;
            add_point_to_trajectory(cur_pos_yaw, glance_traj);
          }

          glance_traj.header.frame_id = m_world_frame_id;
          glance_traj.header.stamp = ros::Time::now();
          glance_traj.use_yaw = true;
          glance_traj.fly_now = true;
      
          m_pub_cmd_traj.publish(glance_traj);
          if (m_pub_dbg_traj.getNumSubscribers() > 0)
            m_pub_dbg_traj.publish(traj_to_path(glance_traj, m_trajectory_sampling_dt));
        }
        
        //}

        const auto time_since_last_det_msg = ros::Time::now() - m_sh_ball_detection->last_message_time();

        /* check if we've seen the ball again and resume huntung if possible //{ */
        
        if (m_sh_ball_detection->has_data() && time_since_last_det_msg < m_max_unseen_dur)
        {
          ROS_WARN_STREAM("[LOST_GLANCING]: Saw the ball, continuing!");
          m_following_start = ros::Time::now();
          m_ball_positions.clear();
          m_state = state_enum::following_detection;
        }
        
        //}

        /* check time since last detection message and abort if it's been too long //{ */
        
        if (m_sh_ball_detection->has_data() && time_since_last_det_msg > m_max_unseen_dur + m_glancing_dur)
        {
          ROS_WARN_STREAM("[LOST_GLANCING]: Lost ball. Going back to start!");
          m_state = state_enum::waiting_for_detection;
        }
        
        //}

        //}
      }
      break;
      case state_enum::following_detection:
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "[STATEMACH]: Current state: 'FOLLOWING_DETECTION'");
        /*  //{ */

        /* the main action //{ */
        
        const auto cur_cmd_pos_yaw_opt = get_current_cmd_position();
        if (m_sh_ball_detection->new_data() && cur_cmd_pos_yaw_opt.has_value())
        {
          const auto ball_filtered = *(m_sh_ball_detection->get_data());
          const vec4_t& cur_cmd_pos_yaw = cur_cmd_pos_yaw_opt.value();
          const vec3_t& cur_cmd_pos = cur_cmd_pos_yaw.block<3, 1>(0, 0);
          const vec3_t ball_pos(ball_filtered.detection.pose.position.x, ball_filtered.detection.pose.position.y, ball_filtered.detection.pose.position.z);
          const vec3_t dir_vec = (ball_pos - cur_cmd_pos).normalized();
          const double yaw = std::atan2(dir_vec.y(), dir_vec.x());
          const vec3_t offset_vec = m_target_offset*calc_horizontal_offset_vector(dir_vec);
          const vec3_t tgt_pos = ball_pos + offset_vec;
      
          auto follow_traj = sample_trajectory_between_pts(cur_cmd_pos, tgt_pos, m_approach_speed, m_trajectory_sampling_dt, yaw);
          const auto follow_traj_duration = trajectory_duration(follow_traj.points.size(), m_trajectory_sampling_dt);
          ROS_INFO_STREAM_THROTTLE(1.0, "[FOLLOWING_DETECTION]: Follow trajectory: " << follow_traj_duration.toSec() << "s, " << follow_traj.points.size() << "pts");
      
          follow_traj.header.frame_id = m_world_frame_id;
          follow_traj.header.stamp = ros::Time::now();
          follow_traj.use_yaw = true;
          follow_traj.fly_now = true;
      
          m_pub_cmd_traj.publish(follow_traj);
          if (m_pub_dbg_traj.getNumSubscribers() > 0)
            m_pub_dbg_traj.publish(traj_to_path(follow_traj, m_trajectory_sampling_dt));

          m_ball_positions.push_back(ball_pos);
        }
        
        //}

        /* check if some prediction is available and if so, change the state //{ */
        
        if (lkf_valid || ukf_valid)
        {
          ROS_WARN_STREAM("[FOLLOWING_DETECTION]: Got a prediction, changing state to following prediction!");
          m_state = state_enum::following_prediction;
        }
        
        //}

        /* check if we've been following for long enough to start lurking //{ */
        
        const auto following_dur = ros::Time::now() - m_following_start;
        ROS_WARN_THROTTLE(1.0, "[FOLLOWING_PREDICTION]: Following for %.2f/%.2fs with %lu/%d points!", following_dur.toSec(), m_lurking_min_dur.toSec(), m_ball_positions.size(), m_lurking_min_pts);
        if (following_dur >= m_lurking_min_dur && (int)m_ball_positions.size() > m_lurking_min_pts)
        {
          ROS_WARN_STREAM("[FOLLOWING_PREDICTION]: Following duration fulfilled, changing state to chasing prediction!");
          m_state = state_enum::going_to_lurk;
        }
        
        //}

        /* check time since last detection message and abort if it's been too long //{ */
        
        {
          const auto time_since_last_det_msg = ros::Time::now() - m_sh_ball_detection->last_message_time();
          if (m_sh_ball_detection->has_data() && time_since_last_det_msg > m_max_unseen_dur)
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

        /* the main action //{ */
        
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
            std_msgs::Header header;
            header.frame_id = m_world_frame_id;
            header.stamp = cur_time;
            // if the flow gets here, then ball_prediction.predicted_path.poses has at least one element
            const auto ball_pos_ros = pred_path.poses.front().pose.position;
            const vec3_t ball_pos(ball_pos_ros.x, ball_pos_ros.y, ball_pos_ros.z);
            const double ball_expected_speed = ball_prediction.filter_state.expected_speed;
        
            auto follow_traj_pre = sample_trajectory_between_pts(cur_cmd_pos, ball_pos, m_approach_speed, m_trajectory_sampling_dt, 0.0, header);
            const auto follow_traj_pre_duration = trajectory_duration(follow_traj_pre.points.size(), m_trajectory_sampling_dt);
            ROS_INFO_STREAM_THROTTLE(1.0, "[FOLLOWING_PREDICTION]: Approach trajectory: " << follow_traj_pre_duration.toSec() << "s, " << follow_traj_pre.points.size() << "pts");
            follow_traj_pre = orient_trajectory_yaw_observe(follow_traj_pre, pred_path);
        
            const int n_pts = m_max_pts - follow_traj_pre.points.size();
            auto follow_traj_post = sample_trajectory_from_path(pred_path, m_trajectory_sampling_dt, ball_expected_speed, n_pts, header);
            const auto follow_traj_post_duration = trajectory_duration(follow_traj_post.points.size(), m_trajectory_sampling_dt);
            ROS_INFO_STREAM_THROTTLE(1.0, "[FOLLOWING_PREDICTION]: Chase trajectory: " << follow_traj_post_duration.toSec() << "s, " << follow_traj_post.points.size() << "pts");
            follow_traj_post = orient_trajectory_yaw_speed(follow_traj_post, pred_path);
        
            traj_t total_traj = join_trajectories(follow_traj_pre, follow_traj_post);
            total_traj.header = header;
            total_traj.use_yaw = true;
            total_traj.fly_now = true;
        
            m_pub_cmd_traj.publish(total_traj);
            if (m_pub_dbg_traj.getNumSubscribers() > 0)
              m_pub_dbg_traj.publish(traj_to_path(total_traj, m_trajectory_sampling_dt));

            m_ball_positions.push_back(ball_pos);
          }
        
        }
        
        //}

        /* check if we've been following for long enough to start lurking //{ */
        
        const auto following_dur = ros::Time::now() - m_following_start;
        ROS_WARN_THROTTLE(1.0, "[FOLLOWING_PREDICTION]: Following for %.2f/%.2fs with %lu/%d points!", following_dur.toSec(), m_lurking_min_dur.toSec(), m_ball_positions.size(), m_lurking_min_pts);
        if (following_dur >= m_lurking_min_dur && (int)m_ball_positions.size() > m_lurking_min_pts)
        {
          ROS_WARN_STREAM("[FOLLOWING_PREDICTION]: Following duration fulfilled, changing state to chasing prediction!");
          m_state = state_enum::going_to_lurk;
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
      case state_enum::going_to_lurk:
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "[STATEMACH]: Current state: 'GOING_TO_LURK'");
        /*  //{ */

        {
          const auto lurking_pose = choose_lurking_pose(m_ball_positions);
          m_orig_lurk_pose = lurking_pose;

          traj_t intercept_traj;
          add_point_to_trajectory(lurking_pose, intercept_traj);
          intercept_traj.header.frame_id = m_world_frame_id;
          intercept_traj.header.stamp = ros::Time::now();
          intercept_traj.use_yaw = true;
          intercept_traj.fly_now = true;
          m_pub_cmd_traj.publish(intercept_traj);
          if (m_pub_dbg_traj.getNumSubscribers() > 0)
            m_pub_dbg_traj.publish(traj_to_path(intercept_traj, m_trajectory_sampling_dt));

          ROS_INFO_THROTTLE(1.0, "[GOING_TO_LURK]: Going to lurk at position [%.2f, %.2f, %.2f] (yaw: %.2f)", lurking_pose.x(), lurking_pose.y(), lurking_pose.z(), lurking_pose.w());

          // wait for the lurker to get into the lurking position
          if (m_sh_tracker_diags->has_data() && !m_sh_tracker_diags->get_data()->tracking_trajectory)
          {
            ROS_INFO("[GOING_TO_LURK]: At lurking spot, switching state to 'LURKING'.");
            reset_filter();
            // reset detections and predictions
            if (m_sh_ball_detection->new_data())
              m_sh_ball_detection->get_data();
            if (m_sh_ball_prediction->new_data())
              m_sh_ball_prediction->get_data();
            // move to the next state
            m_state = state_enum::lurking;
          }
        }

        //}
      }
      break;
      case state_enum::lurking:
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "[STATEMACH]: Current state: 'LURKING'");
        /*  //{ */

        const auto cur_cmd_pos_yaw_opt = get_current_cmd_position();
        if (cur_cmd_pos_yaw_opt.has_value())
        {
          const auto cur_cmd_pos_yaw = cur_cmd_pos_yaw_opt.value();
          const auto cur_cmd_pos = cur_cmd_pos_yaw.block<3, 1>(0, 0);
          const auto cur_time = ros::Time::now();

          std::optional<vec4_t> intercept_pos_opt;
          if (m_sh_ball_prediction->new_data())
          /* if prediction is available, adapt the position accordingly //{ */
          {
            const auto ball_prediction = *(m_sh_ball_prediction->get_data());
            const auto& pred_path = ball_prediction.predicted_path;
            if (pred_path.poses.empty())
            {
              ROS_WARN_THROTTLE(1.0, "[LURKING]: Empty prediction received, skipping.");
              break;
            }
            const auto intersect_plane = get_yz_plane(cur_cmd_pos, cur_cmd_pos_yaw(3));
            auto intercept_point = path_plane_intersection(pred_path, intersect_plane);
          
            const auto ball_pos = to_eigen(pred_path.poses.front().pose.position);
            const vec3_t dir_vec = (ball_pos - cur_cmd_pos).normalized();
            const double yaw = std::atan2(dir_vec.y(), dir_vec.x());
            const vec4_t intercept_pos(intercept_point.x(), intercept_point.y(), intercept_point.z(), yaw);
            intercept_pos_opt = intercept_pos;
          }
          //}
          if (!intercept_pos_opt.has_value() && m_sh_ball_detection->new_data())
          /* otherwise use the detection (if available) to orient the lurker //{ */
          {
            const auto ball_detection = *(m_sh_ball_detection->get_data());
            const auto ball_pos = to_eigen(ball_detection.detection.pose.position);
            const vec3_t dir_vec = (ball_pos - cur_cmd_pos).normalized();
            const double yaw = std::atan2(dir_vec.y(), dir_vec.x());
            intercept_pos_opt = cur_cmd_pos_yaw;
            intercept_pos_opt.value()(3) = yaw;
          }
          //}

          /* publish the new command trajectory //{ */
          
          if (intercept_pos_opt.has_value())
          {
            auto lurking_pose = intercept_pos_opt.value();
            const auto lurking_point = lurking_pose.block<3, 1>(0, 0);
            // limit the maximal reposition to m_lurking_max_reposition from the original lurk position
            const auto reposition = lurking_point - m_orig_lurk_pose.block<3, 1>(0, 0);
            const auto repos_dist = reposition.norm();
            /* ROS_INFO("[]: repos_dist: %.2f, max repos: %.2f", repos_dist, m_lurking_max_reposition); */
            if (repos_dist > m_lurking_max_reposition)
              lurking_pose.block<3, 1>(0, 0) = m_orig_lurk_pose.block<3, 1>(0, 0) + reposition/repos_dist*m_lurking_max_reposition;
          
            traj_t intercept_traj;
            add_point_to_trajectory(lurking_pose, intercept_traj);
            intercept_traj.header.frame_id = m_world_frame_id;
            intercept_traj.header.stamp = cur_time;
            intercept_traj.use_yaw = true;
            intercept_traj.fly_now = true;
            m_pub_cmd_traj.publish(intercept_traj);
            if (m_pub_dbg_traj.getNumSubscribers() > 0)
              m_pub_dbg_traj.publish(traj_to_path(intercept_traj, m_trajectory_sampling_dt));
          
            const auto repos_dist2 = (lurking_pose.block<3, 1>(0, 0) - m_orig_lurk_pose.block<3, 1>(0, 0)).norm();
            ROS_INFO_THROTTLE(1.0, "[GOING_TO_LURK]: Lurking at position [%.2f, %.2f, %.2f] (yaw: %.2f) - %.2fm from orig. lurk point", lurking_pose.x(), lurking_pose.y(), lurking_pose.z(), lurking_pose.w(), repos_dist2);
          }
          
          //}
        }

        //}
      }
      break;
    }

    /* publish remembered ball positions for debugging //{ */
    
    if (m_pub_dbg_ball_positions.getNumSubscribers())
    {
      std_msgs::Header header;
      header.frame_id = m_world_frame_id;
      header.stamp = ros::Time::now();
      m_pub_dbg_ball_positions.publish(to_output_message(m_ball_positions, header));
    }
    
    //}
  }
  //}

  // --------------------------------------------------------------
  // |                       Helper methods                       |
  // --------------------------------------------------------------

    /* pick_constraints() method //{ */
    std::string BalloonPlanner::pick_constraints(const double ball_dist)
    {
      assert(!m_constraint_ranges.empty());
      auto prev_constraint = std::begin(m_constraint_ranges)->second;
      for (const auto& keyval : m_constraint_ranges)
      {
        if (keyval.first > ball_dist)
          return prev_constraint;
        prev_constraint = keyval.second;
      }
      return prev_constraint;
    }
    //}

    /* set_constraints() method //{ */
    void BalloonPlanner::set_constraints(const std::string& constraints_name)
    {
      mrs_msgs::String::Request req;
      mrs_msgs::String::Response res;
      req.value = constraints_name;
      if (m_srv_set_constraints.call(req, res))
        ROS_INFO("Gain manager response: %s", res.message.c_str());
      else
        ROS_ERROR("Failed to call service to set the constraints!");
    }
    //}

    /* reset_filter() method //{ */
    void BalloonPlanner::reset_filter()
    {
      balloon_filter::ResetEstimatesRequest req;
      balloon_filter::ResetEstimatesResponse res;
      if (m_srv_reset_filter.call(req, res))
        ROS_INFO("Filter response: %s", res.message.c_str());
      else
        ROS_ERROR("Failed to call service to reset the filter!");
    }
    //}

    /* choose_lurking_pose() method //{ */
    vec4_t BalloonPlanner::choose_lurking_pose(const std::vector<vec3_t>& ball_positions)
    {
      assert(ball_positions.size() > 1);

      const vec3_t lurk_pos = ball_positions.back();
      vec3_t observe_pos = ball_positions.front();
      double observe_dist = std::numeric_limits<double>::max();
      for (const auto& pos : ball_positions)
      {
        const double cur_dist = (pos - lurk_pos).norm();
        if (std::abs(cur_dist - m_lurking_observe_dist) < (observe_dist - m_lurking_observe_dist))
        {
          observe_pos = pos;
          observe_dist = cur_dist;
        }
      }
      const vec3_t lurk_dir = observe_pos - lurk_pos;
      const double lurk_yaw = std::atan2(lurk_dir.y(), lurk_dir.x());
      const vec4_t lurk_pose(lurk_pos.x(), lurk_pos.y(), lurk_pos.z(), lurk_yaw);


      if (m_pub_dbg_lurking_points.getNumSubscribers())
      {
        std_msgs::Header header;
        header.frame_id = m_world_frame_id;
        header.stamp = ros::Time::now();
        std::vector<vec3_t> lurk_pts {observe_pos, lurk_pos};
        m_pub_dbg_lurking_points.publish(to_output_message(lurk_pts, header));
      }

      return lurk_pose;
    }
    //}

    /* get_yz_plane() method //{ */
    plane_t BalloonPlanner::get_yz_plane(const vec3_t& pos, const double yaw)
    {
      const vec3_t normal(cos(yaw), sin(yaw), 0.0);
      plane_t ret;
      ret.point = pos;
      ret.normal = normal;
      return ret;
    }
    //}

    /* line_plane_intersection_coeff() //{ */
    double line_plane_intersection_coeff(const vec3_t& line_start, const vec3_t& line_dir, const plane_t& plane)
    {
      const double den = line_dir.dot(plane.normal);
      if (den == 0.0)
        return std::numeric_limits<double>::max();
      const double nom = (plane.point - line_start).dot(plane.normal);
      return nom/den;
    }
    //}

    /* path_plane_intersection() method //{ */
    vec3_t BalloonPlanner::path_plane_intersection(const path_t& path, const plane_t& plane)
    {
      assert(path.poses.size() > 1);
      std::optional<vec3_t> intersection;
      vec3_t prev_end = to_eigen(path.poses.front().pose.position);
      for (const auto& pose : path.poses)
      {
        const vec3_t line_start = prev_end;
        const vec3_t line_end = to_eigen(pose.pose.position);
        const vec3_t line_dir = line_end - line_start;
        const double cur_coeff = line_plane_intersection_coeff(line_start, line_dir, plane);
        if (cur_coeff >= 0.0 && cur_coeff <= 1.0)
          intersection = line_start + cur_coeff*line_dir;
        prev_end = line_end;
      }
      if (!intersection.has_value())
      {
        const vec3_t line_start = to_eigen(path.poses.at(path.poses.size()-2).pose.position);
        const vec3_t line_end = prev_end;
        const vec3_t line_dir = line_end - line_start;
        const double cur_coeff = line_plane_intersection_coeff(line_start, line_dir, plane);
        intersection = line_start + cur_coeff*line_dir;
      }
      return intersection.value();
    }
    //}

  /* yaw_from_quat() method //{ */
  double yaw_from_quat(const quat_t& quat)
  {
    const vec3_t unit_x_rotated = quat*vec3_t::UnitX();
    const double yaw = std::atan2(unit_x_rotated.y(), unit_x_rotated.x());
    return yaw;
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

  /* trajectory_duration() method //{ */
  ros::Duration BalloonPlanner::trajectory_duration(const int n_traj_pts, const double dt)
  {
    const ros::Duration traj_dur(n_traj_pts > 0 ? (n_traj_pts - 1) * dt : 0);
    return traj_dur;
  }
  //}

  /* sample_trajectory_between_pts() method //{ */
  traj_t BalloonPlanner::sample_trajectory_between_pts(const vec3_t& from_pt, const vec3_t& to_pt, const double speed,
                                                                                  const double dt, const double yaw,
                                                                                  const std_msgs::Header& header)
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

    ret.header = header;
    return ret;
  }
  //}

  /* sample_trajectory_from_path() method //{ */
  traj_t BalloonPlanner::sample_trajectory_from_path(const path_t& path, const double dt, const double speed, const size_t n_pts, const std_msgs::Header& header)
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

    ret.header = header;
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

      const auto cur_pt = linear_interpolation(cur_stamp, prev_pose, next_pose);
      add_point_to_trajectory(cur_pt, ret);

      cur_stamp += dt_dur;
    }

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

  /* to_output_message() method //{ */
  sensor_msgs::PointCloud2 BalloonPlanner::to_output_message(const std::vector<vec3_t>& points, const std_msgs::Header& header)
  {
    const size_t n_pts = points.size();
    sensor_msgs::PointCloud2 ret;
    ret.header = header;
    ret.height = 1;
    ret.width = n_pts;

    {
      // Prepare the PointCloud2
      sensor_msgs::PointCloud2Modifier modifier(ret);
      modifier.setPointCloud2FieldsByString(1, "xyz");
      modifier.resize(n_pts);
    }

    {
      // Fill the PointCloud2
      sensor_msgs::PointCloud2Iterator<float> iter_x(ret, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(ret, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(ret, "z");
      for (size_t it = 0; it < n_pts; it++, ++iter_x, ++iter_y, ++iter_z)
      {
        *iter_x = points.at(it).x();
        *iter_y = points.at(it).y();
        *iter_z = points.at(it).z();
      }
    }

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
    pl.load_param("max_unseen_duration", m_max_unseen_dur);
    pl.load_param("glancing/duration", m_glancing_dur);
    pl.load_param("glancing/yaw_rate", m_glancing_yaw_rate);

    pl.load_param("trajectory/sampling_dt", m_trajectory_sampling_dt);

    pl.load_param("lurking/min_follow_duration", m_lurking_min_dur);
    pl.load_param("lurking/min_points", m_lurking_min_pts);
    pl.load_param("lurking/observe_dist", m_lurking_observe_dist);
    pl.load_param("lurking/max_dist_from_trajectory", m_lurking_max_dist_from_trajectory);
    pl.load_param("lurking/max_reposition", m_lurking_max_reposition);

    /* load and print constraint ranges //{ */
    
    const auto constraint_ranges_unsorted = pl.load_param2<std::map<std::string, double>>("constraint_ranges");
    for (const auto& keyval : constraint_ranges_unsorted)
      m_constraint_ranges.emplace(keyval.second, keyval.first);
    ROS_INFO("[%s]: Sorted constraint ranges:", m_node_name.c_str());
    std::cout << "{" <<std::endl;
    double prev_dist = 0.0;
    for (const auto& keyval : m_constraint_ranges)
    {
      std::cout << "\t" << prev_dist << "~" << keyval.first << ":\t" << keyval.second << std::endl;
      prev_dist = keyval.first;
    }
    std::cout << "}" <<std::endl;
    
    //}

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
    m_sh_tracker_diags = smgr.create_handler<mrs_msgs::MpcTrackerDiagnostics>("tracker_diagnostics", ros::Duration(5.0));

    //}

    /* publishers //{ */

    m_pub_cmd_traj = nh.advertise<mrs_msgs::TrackerTrajectory>("commanded_trajectory", 1);
    m_pub_dbg_traj = nh.advertise<nav_msgs::Path>("debug_trajectory", 1);
    m_pub_dbg_ball_positions = nh.advertise<sensor_msgs::PointCloud2>("ball_positions", 1);
    m_pub_dbg_lurking_points = nh.advertise<sensor_msgs::PointCloud2>("lurking_points", 1, true);

    //}

    /* services //{ */
    
    m_srv_reset_filter = nh.serviceClient<balloon_filter::ResetEstimates>("reset_balloon_filter");
    m_srv_set_constraints = nh.serviceClient<mrs_msgs::String>("set_constraints");
    
    //}

    /* profiler //{ */

    m_profiler_ptr = std::make_unique<mrs_lib::Profiler>(nh, m_node_name, false);

    //}

    /* timers  //{ */

    m_main_loop_timer = nh.createTimer(ros::Duration(planning_period), &BalloonPlanner::main_loop, this);

    //}

    m_ball_positions.reserve(200);
    m_state = state_enum::waiting_for_detection;

    ROS_INFO("[%s]: initialized", m_node_name.c_str());
  }

  //}

}  // namespace balloon_planner

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(balloon_planner::BalloonPlanner, nodelet::Nodelet)
