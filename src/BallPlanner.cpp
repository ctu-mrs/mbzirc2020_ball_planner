#include <ball_planner/BallPlanner.h>

namespace ball_planner
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
    mrs_msgs::Reference tr_pt;
    tr_pt.position.x = pt.x();
    tr_pt.position.y = pt.y();
    tr_pt.position.z = pt.z();
    traj.points.push_back(tr_pt);
  }

  void add_point_to_trajectory(const vec4_t pt, traj_t& traj)
  {
    mrs_msgs::Reference tr_pt;
    tr_pt.position.x = pt.x();
    tr_pt.position.y = pt.y();
    tr_pt.position.z = pt.z();
    tr_pt.heading = pt.w();
    traj.points.push_back(tr_pt);
  }
  //}

  /* main_loop() method //{ */
  void BallPlanner::main_loop([[maybe_unused]] const ros::TimerEvent& evt)
  {
    load_dynparams(m_drmgr_ptr->config);

    if (!m_initialized)
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "[STATEMACH]: NOT INITIALIZED");
      return;
    }

    if (m_land)
    {
      ROS_ERROR_STREAM("[STATEMACH]: LANDING!!!!");
      land_there(m_landing_pose);
      return;
    }

    if (!m_activated)
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "[STATEMACH]: INACTIVE");
      return;
    }

    const auto cur_pos_yaw_opt = get_uav_position();
    const auto ball_pose_stamped_opt = get_ball_position();
    const auto ball_passthrough_stamped_opt = get_ball_passthrough();
    const auto ball_pred_opt = get_ball_prediction();
    /* set constraints, set last seen yaw etc. according to ball position if available //{ */

    {
      const auto desired_constraints = pick_constraints(m_state);
      /* ROS_INFO_STREAM_THROTTLE(1.0, "[BallPlanner]: Setting '" << desired_constraints << "' constraints."); */
      set_constraints(desired_constraints);
    }

    //}

    switch (m_state)
    {
      case state_enum::going_to_nextpos_low:
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "[STATEMACH]: Current state: 'GOINT_TO_NEXTPOS_LOW'");
        /*  //{ */
        
        m_cur_goto_pose = m_start_pose;
        m_cur_goto_pose.z() = m_pogo_min_height;
        traj_t result_traj;
        result_traj.header.frame_id = m_world_frame_id;
        result_traj.header.stamp = ros::Time(0);  // just fly now
        result_traj.use_heading = true;
        result_traj.fly_now = true;
        add_point_to_trajectory(m_cur_goto_pose, result_traj);
        m_pub_cmd_traj.publish(result_traj);
        if (m_pub_dbg_traj.getNumSubscribers() > 0)
          m_pub_dbg_traj.publish(traj_to_path(result_traj, m_trajectory_sampling_dt));
        
        ROS_INFO_STREAM_THROTTLE(1.0, "[GOINT_TO_NEXTPOS_LOW]: Going to start point (at low height) [" << m_cur_goto_pose.transpose() << "]");
        if (cur_pos_yaw_opt.has_value())
        {
          const auto cur_pos_yaw = cur_pos_yaw_opt.value();
          const auto cur_pos = cur_pos_yaw.block<3, 1>(0, 0);
          const auto goto_pos = m_cur_goto_pose.block<3, 1>(0, 0);
          const double des_pos_dist = (goto_pos - cur_pos).norm();
        
          ROS_INFO_STREAM_THROTTLE(1.0, "[GOINT_TO_NEXTPOS_LOW]: Distance from start point (at low height): " << des_pos_dist << "m/" << m_trajectory_tgt_reached_dist << "m.");
          if (des_pos_dist < m_trajectory_tgt_reached_dist)
          {
            ROS_WARN_STREAM("[GOINT_TO_NEXTPOS_LOW]: Reached target position, continuing!");

            // reset filter and detector
            reset_detector();
            reset_filter();

            m_state = state_enum::going_to_nextpos;
          }
        }
        
        //}
      }
      break;
      case state_enum::going_to_nextpos:
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "[STATEMACH]: Current state: 'GOINT_TO_NEXTPOS'");
        /*  //{ */
        
        m_cur_goto_pose = m_start_pose;
        traj_t result_traj;
        result_traj.header.frame_id = m_world_frame_id;
        result_traj.header.stamp = ros::Time(0);  // just fly now
        result_traj.use_heading = true;
        result_traj.fly_now = true;
        add_point_to_trajectory(m_cur_goto_pose, result_traj);
        m_pub_cmd_traj.publish(result_traj);
        if (m_pub_dbg_traj.getNumSubscribers() > 0)
          m_pub_dbg_traj.publish(traj_to_path(result_traj, m_trajectory_sampling_dt));
        
        ROS_INFO_STREAM_THROTTLE(1.0, "[GOINT_TO_NEXTPOS]: Going to start point [" << m_cur_goto_pose.transpose() << "]");
        if (cur_pos_yaw_opt.has_value())
        {
          const auto cur_pos_yaw = cur_pos_yaw_opt.value();
          const auto cur_pos = cur_pos_yaw.block<3, 1>(0, 0);
          const auto goto_pos = m_cur_goto_pose.block<3, 1>(0, 0);
          const double des_pos_dist = (goto_pos - cur_pos).norm();
        
          ROS_INFO_STREAM_THROTTLE(1.0, "[GOINT_TO_NEXTPOS]: Distance from start point: " << des_pos_dist << "m/" << m_trajectory_tgt_reached_dist << "m.");
          if (des_pos_dist < m_trajectory_tgt_reached_dist)
          {
            ROS_WARN_STREAM("[GOINT_TO_NEXTPOS]: Reached target position, continuing!");
            m_pogo_prev_time = ros::Time::now();
            m_pogo_prev_height = m_cur_goto_pose.z();
            m_pogo_direction = 1.0;
            m_state = state_enum::waiting_for_detection;
          }
        }
        
        //}
      }
      break;
      case state_enum::waiting_for_detection:
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "[STATEMACH]: Current state: 'WAITING_FOR_DETECTION'");
        /*  //{ */

        // | ------------------- POGO POGO POGO POGO ------------------ |
        const double pogo_dt = (ros::Time::now() - m_pogo_prev_time).toSec();
        const double pogo_height = std::clamp(m_pogo_prev_height + m_pogo_direction*pogo_dt*m_pogo_speed, m_pogo_min_height, m_pogo_max_height);
        m_pogo_prev_height = pogo_height;
        m_pogo_prev_time = ros::Time::now();
        if (pogo_height == m_pogo_min_height)
          m_pogo_direction = 1.0;
        else if (pogo_height == m_pogo_max_height)
          m_pogo_direction = -1.0;
        ROS_INFO_STREAM_THROTTLE(1.0, "[WAITING_FOR_DETECTION]: Waiting for detection at pogo height: " << pogo_height << "m from [" << m_pogo_min_height << "," << m_pogo_max_height << "], direction: " << m_pogo_direction);

        const vec4_t new_pose(m_start_pose.x(), m_start_pose.y(), pogo_height, m_start_pose.w());
        traj_t result_traj;
        result_traj.header.frame_id = m_world_frame_id;
        result_traj.header.stamp = ros::Time(0);  // just fly now
        result_traj.use_heading = true;
        result_traj.fly_now = true;
        add_point_to_trajectory(new_pose, result_traj);
        m_pub_cmd_traj.publish(result_traj);
        if (m_pub_dbg_traj.getNumSubscribers() > 0)
          m_pub_dbg_traj.publish(traj_to_path(result_traj, m_trajectory_sampling_dt));

        const auto time_since_last_det_msg = ros::Time::now() - m_sh_ball_detection.lastMsgTime();
        const bool det_valid = m_sh_ball_detection.hasMsg() && time_since_last_det_msg < m_max_unseen_dur;
        const auto time_since_last_passthrough_msg = ros::Time::now() - m_sh_ball_passthrough.lastMsgTime();
        const bool passthrough_valid = m_sh_ball_passthrough.hasMsg() && time_since_last_passthrough_msg < m_max_unseen_dur;
        if (det_valid || passthrough_valid)
        {
          ROS_WARN_STREAM("[WAITING_FOR_DETECTION]: Saw the ball, continuing!");
          m_observing_start = ros::Time::now();
          m_ball_positions.clear();
          m_state = state_enum::observing;
        }

        //}
      }
      break;
      case state_enum::observing:
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "[STATEMACH]: Current state: 'OBSERVING'");
        /*  //{ */

        const auto now = ros::Time::now();
        const auto observing_dur = now - m_observing_start;
        const auto time_since_last_det_msg = now - m_sh_ball_detection.lastMsgTime();

        vec4_t observe_pose = m_start_pose;
        // adjust the height according to the detection, if available
        if (ball_pose_stamped_opt.has_value())
        {
          const vec4_t ball_pose = ball_pose_stamped_opt.value().pose;
          observe_pose.z() = ball_pose.z();
          /* if (cur_pos_yaw_opt.has_value()) */
          /* { */
          /*   const vec3_t cur_pos = cur_pos_yaw_opt.value().block<3, 1>(0, 0); */
          /*   const vec3_t diff_vec = ball_pose.block<3, 1>(0, 0) - cur_pos; */
          /*   const double yaw = std::atan2(diff_vec.y(), diff_vec.x()); */
          /*   observe_pose.w() = yaw; */
          /* } */
          ROS_INFO_THROTTLE(1.0, "[OBSERVING]: Observing for %.2fs/%.2fs. Last seen before %.2fs. Using detection height of %.2fm.", observing_dur.toSec(), m_lurking_min_observing_dur.toSec(),
                            time_since_last_det_msg.toSec(), observe_pose.z());
        }
        // otherwise use the last passthrough height
        else if (ball_passthrough_stamped_opt.has_value())
        {
          observe_pose.z() = ball_passthrough_stamped_opt.value().pose.z();
          ROS_INFO_THROTTLE(1.0, "[OBSERVING]: Observing for %.2fs/%.2fs. Last seen before %.2fs. Using passthrough height of %.2fm.", observing_dur.toSec(), m_lurking_min_observing_dur.toSec(),
                            time_since_last_det_msg.toSec(), observe_pose.z());
        }
        else
        {
          ROS_INFO_THROTTLE(1.0, "[OBSERVING]: Observing for %.2fs/%.2fs. Last seen before %.2fs. Using default height of %.2fm.", observing_dur.toSec(), m_lurking_min_observing_dur.toSec(),
                            time_since_last_det_msg.toSec(), observe_pose.z());
        }

        traj_t result_traj;
        result_traj.header.frame_id = m_world_frame_id;
        result_traj.header.stamp = ros::Time(0);  // just fly now
        result_traj.use_heading = true;
        result_traj.fly_now = true;
        add_point_to_trajectory(observe_pose, result_traj);
        m_pub_cmd_traj.publish(result_traj);
        if (m_pub_dbg_traj.getNumSubscribers() > 0)
          m_pub_dbg_traj.publish(traj_to_path(result_traj, m_trajectory_sampling_dt));

        if (observing_dur >= m_lurking_min_observing_dur)
        {
          if (ball_passthrough_stamped_opt.has_value())
          {
            // calculate the luring pose where we want to be
            vec4_t lurking_pose = ball_passthrough_stamped_opt.value().pose;
            lurking_pose.w() += M_PI;                // rotate the yaw to face the incoming ball
            m_orig_lurk_pose = lurking_pose;         // this member variable should not change throughout the lurking
            m_cur_lurk_pose = lurking_pose;          // this will change in adaptation to the predicted ball trajectory
            lurking_pose.z() += m_lurking_z_offset;  // apply the desired offset to the z coordinate
            m_cur_lurk_pose_offset = lurking_pose;   // this is the current lurk pose, properly offset in the z coordinate

            ROS_WARN_STREAM("[OBSERVING]: Observing duration fulfilled, passthrough pose available, going to lurk!");
            m_state = state_enum::going_to_lurk_down;
          }
          else
          {
            ROS_WARN_STREAM("[OBSERVING]: Observing duration fulfilled, passthrough pose NOT available, cannot go lurking!");
          }
        }

        //}
      }
      break;
      case state_enum::going_to_lurk_down:
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "[STATEMACH]: Current state: 'GOING_TO_LURK_DOWN'");
        /*  //{ */

        if (cur_pos_yaw_opt.has_value())
        {
          const auto cur_pos = cur_pos_yaw_opt.value().block<3, 1>(0, 0);
          const auto cur_yaw = cur_pos_yaw_opt.value().w();

          // calculate the pose where we want to be
          std_msgs::Header header;
          header.frame_id = m_world_frame_id;
          header.stamp = ros::Time::now();
          m_cur_goto_pose = {cur_pos.x(), cur_pos.y(), m_pogo_min_height, cur_yaw};

          // go to minimal flight height
          traj_t result_traj;
          result_traj.header = header;
          result_traj.header.stamp = ros::Time(0);  // just fly now
          result_traj.use_heading = true;
          result_traj.fly_now = true;
          add_point_to_trajectory(m_cur_goto_pose, result_traj);

          m_pub_cmd_traj.publish(result_traj);
          if (m_pub_dbg_traj.getNumSubscribers() > 0)
            m_pub_dbg_traj.publish(traj_to_path(result_traj, m_trajectory_sampling_dt));

          ROS_INFO_THROTTLE(1.0, "[GOING_TO_LURK_DOWN]: Going to min-height position [%.2f, %.2f, %.2f] (yaw: %.2f). Detection position: [%.2f, %.2f, %.2f]",
                            m_cur_goto_pose.x(), m_cur_goto_pose.y(), m_cur_goto_pose.z(), m_cur_goto_pose.w(), m_orig_lurk_pose.x(), m_orig_lurk_pose.y(),
                            m_orig_lurk_pose.z());
        }

        /* check if we got to the desired position yet //{ */
        
        if (cur_pos_yaw_opt.has_value())
        {
          // wait for the lurker to get into the lurking position
          const auto cur_pos = cur_pos_yaw_opt.value().block<3, 1>(0, 0);
          const double cur_dist = (m_cur_goto_pose.block<3, 1>(0, 0) - cur_pos).norm();
          if (cur_dist < m_trajectory_tgt_reached_dist)
          {
            ROS_INFO("[GOING_TO_LURK_DOWN]: At lowest-z height, switching state to 'GOING_TO_LURK_HORIZONTAL'.");
            // move to the next state
            m_state = state_enum::going_to_lurk_horizontal;
          } else
          {
            ROS_INFO_THROTTLE(1.0, "[GOING_TO_LURK_DOWN]: Waiting to reach lowest-z height position [%.2f, %.2f, %.2f] (yaw: %.2f). Cur. dist.: %.2f.", m_cur_goto_pose.x(),
                              m_cur_goto_pose.y(), m_cur_goto_pose.z(), m_cur_goto_pose.w(), cur_dist);
          }
        }
        
        //}

        //}
      }
      break;
      case state_enum::going_to_lurk_horizontal:
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "[STATEMACH]: Current state: 'GOING_TO_LURK_HORIZONTAL'");
        /*  //{ */

        if (cur_pos_yaw_opt.has_value())
        {
          const auto cur_yaw = cur_pos_yaw_opt.value().w();

          // calculate the pose where we want to be
          std_msgs::Header header;
          header.frame_id = m_world_frame_id;
          header.stamp = ros::Time::now();
          m_cur_goto_pose = {m_cur_lurk_pose_offset.x(), m_cur_lurk_pose_offset.y(), m_pogo_min_height, cur_yaw};

          // go to minimal flight height
          traj_t result_traj;
          result_traj.header = header;
          result_traj.header.stamp = ros::Time(0);  // just fly now
          result_traj.use_heading = true;
          result_traj.fly_now = true;
          add_point_to_trajectory(m_cur_goto_pose, result_traj);

          m_pub_cmd_traj.publish(result_traj);
          if (m_pub_dbg_traj.getNumSubscribers() > 0)
            m_pub_dbg_traj.publish(traj_to_path(result_traj, m_trajectory_sampling_dt));

          ROS_INFO_THROTTLE(1.0, "[GOING_TO_LURK_HORIZONTAL]: Going to min-height lurk position [%.2f, %.2f, %.2f] (yaw: %.2f). Detection position: [%.2f, %.2f, %.2f]",
                            m_cur_goto_pose.x(), m_cur_goto_pose.y(), m_cur_goto_pose.z(), m_cur_goto_pose.w(), m_orig_lurk_pose.x(), m_orig_lurk_pose.y(),
                            m_orig_lurk_pose.z());
        }

        /* check if we got to the desired position yet //{ */
        
        if (cur_pos_yaw_opt.has_value())
        {
          // wait for the lurker to get into the lurking position
          const auto cur_pos = cur_pos_yaw_opt.value().block<3, 1>(0, 0);
          const double cur_dist = (m_cur_goto_pose.block<3, 1>(0, 0) - cur_pos).norm();
          if (cur_dist < m_trajectory_tgt_reached_dist)
          {
            ROS_INFO("[GOING_TO_LURK_HORIZONTAL]: At correct position, switching state to 'GOING_TO_LURK_UP'.");
            // move to the next state
            m_state = state_enum::going_to_lurk_up;
          } else
          {
            ROS_INFO_THROTTLE(1.0, "[GOING_TO_LURK_HORIZONTAL]: Waiting to reach correct position [%.2f, %.2f, %.2f] (yaw: %.2f). Cur. dist.: %.2f.", m_cur_goto_pose.x(),
                              m_cur_goto_pose.y(), m_cur_goto_pose.z(), m_cur_goto_pose.w(), cur_dist);
          }
        }
        
        //}

        //}
      }
      break;
      case state_enum::going_to_lurk_up:
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "[STATEMACH]: Current state: 'GOING_TO_LURK_UP'");
        /*  //{ */

        {
          // calculate the pose where we want to be
          std_msgs::Header header;
          header.frame_id = m_world_frame_id;
          header.stamp = ros::Time::now();
          m_cur_goto_pose = m_cur_lurk_pose_offset;

          // go to minimal flight height
          traj_t result_traj;
          result_traj.header = header;
          result_traj.header.stamp = ros::Time(0);  // just fly now
          result_traj.use_heading = true;
          result_traj.fly_now = true;
          add_point_to_trajectory(m_cur_goto_pose, result_traj);

          m_pub_cmd_traj.publish(result_traj);
          if (m_pub_dbg_traj.getNumSubscribers() > 0)
            m_pub_dbg_traj.publish(traj_to_path(result_traj, m_trajectory_sampling_dt));

          ROS_INFO_THROTTLE(1.0, "[GOING_TO_LURK_UP]: Going to the lurk position [%.2f, %.2f, %.2f] (yaw: %.2f). Detection position: [%.2f, %.2f, %.2f]",
                            m_cur_goto_pose.x(), m_cur_goto_pose.y(), m_cur_goto_pose.z(), m_cur_goto_pose.w(), m_orig_lurk_pose.x(), m_orig_lurk_pose.y(),
                            m_orig_lurk_pose.z());
        }

        /* check if we got to the desired position yet //{ */
        
        if (cur_pos_yaw_opt.has_value())
        {
          // wait for the lurker to get into the lurking position
          const auto cur_pos = cur_pos_yaw_opt.value().block<3, 1>(0, 0);
          const double cur_dist = (m_cur_goto_pose.block<3, 1>(0, 0) - cur_pos).norm();
          if (cur_dist < m_trajectory_tgt_reached_dist)
          {
            ROS_INFO("[GOING_TO_LURK_UP]: At lurking position, switching state to 'LURKING'.");
            // reset detections and predictions
            if (m_sh_ball_detection.newMsg())
              m_sh_ball_detection.getMsg();
            if (m_sh_ball_prediction.newMsg())
              m_sh_ball_prediction.getMsg();
            m_prev_signed_ball_dist_set = false;
            // move to the next state
            m_state = state_enum::lurking;
          } else
          {
            ROS_INFO_THROTTLE(1.0, "[GOING_TO_LURK_UP]: Waiting to reach lurking position [%.2f, %.2f, %.2f] (yaw: %.2f). Cur. dist.: %.2f.", m_cur_goto_pose.x(),
                              m_cur_goto_pose.y(), m_cur_goto_pose.z(), m_cur_goto_pose.w(), cur_dist);
          }
        }
        
        //}

        //}
      }
      break;
      case state_enum::lurking:
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "[STATEMACH]: Current state: 'LURKING'");
        /*  //{ */

        std_msgs::Header header;
        header.frame_id = m_world_frame_id;
        header.stamp = ros::Time::now();

        const auto cur_cmd_pos_yaw_opt = get_uav_cmd_position();
        if (cur_cmd_pos_yaw_opt.has_value())
        {
          const auto cur_cmd_pos_yaw = cur_cmd_pos_yaw_opt.value();
          const auto cur_cmd_pos = cur_cmd_pos_yaw.block<3, 1>(0, 0);
          const auto cur_time = ros::Time::now();

          const auto intersect_plane = get_yz_plane(m_orig_lurk_pose);
          if (m_pub_dbg_xy_plane.getNumSubscribers() > 0)
            m_pub_dbg_xy_plane.publish(plane_visualization(intersect_plane, header));

          // the desired interception position
          std::optional<vec4_t> intercept_pos_opt;
          // used to detect whether the ball has already passed through the lurking position
          // if negative then the ball is behind the YZ plane
          std::optional<vec3_t> ball_pos_opt;

          if (ball_pred_opt.has_value() && !ball_pred_opt.value().predicted_path.poses.empty())
          /* if prediction is available, adapt the position accordingly //{ */
          {
            const auto ball_prediction = ball_pred_opt.value();
            const auto& pred_path = ball_prediction.predicted_path;
            const auto pred_path_age = ros::Time::now() - pred_path.header.stamp;
            if (pred_path_age > m_max_unseen_dur)
            {
              ROS_WARN_THROTTLE(1.0, "[LURKING]: Predicted path is too old, ignoring it (age: %.2f > %.2f)", pred_path_age.toSec(), m_max_unseen_dur.toSec());
            } else
            {
              const auto ball_pos = to_eigen(pred_path.poses.front().pose.position);
              /* const double ball_dist = (ball_pos - cur_cmd_pos).norm(); */
              const double ball_dist = (ball_pos - m_cur_lurk_pose_offset.block<3, 1>(0, 0)).norm();
              ball_pos_opt = ball_pos;
              if (ball_dist > m_lurking_reaction_dist)
              {
                ROS_INFO_THROTTLE(1.0, "[LURKING]: Ball is too far (%.2fm > %.2fm), not reacting yet.", ball_dist, m_lurking_reaction_dist);
              } else
              {
                const double signed_ball_dist = signed_point_plane_distance(ball_pos, intersect_plane);
                if (signed_ball_dist < 0.0)
                {
                  ROS_WARN_THROTTLE(1.0, "[LURKING]: Ball is behind YZ plane (signed dist: %.2fm), ignoring prediction.", signed_ball_dist);
                }
                else
                {
                  const auto intercept_point_opt = path_plane_intersection(pred_path, intersect_plane);
                  if (intercept_point_opt.has_value())
                  {
                    ROS_INFO_THROTTLE(1.0, "[LURKING]: Adapting position according to prediction.");
                    const auto intercept_point = intercept_point_opt.value();
                    if (m_pub_dbg_int_pt.getNumSubscribers() > 0)
                      m_pub_dbg_int_pt.publish(to_msg(intercept_point, header));

                    /* const vec3_t dir_vec = (ball_pos - cur_cmd_pos).normalized(); */
                    /* const double yaw = std::atan2(dir_vec.y(), dir_vec.x()); */
                    const vec4_t intercept_pos(intercept_point.x(), intercept_point.y(), intercept_point.z() + m_lurking_z_offset, m_cur_lurk_pose.w());
                    intercept_pos_opt = intercept_pos;
                  } else
                  {
                    ROS_WARN_THROTTLE(1.0, "[LURKING]: Predicted trajectory doesn't intersect the XY lurking plane! Ignoring it.");
                  }
                }
              }
            }
          }
          //}
          /* if (!intercept_pos_opt.has_value() && ball_pose_stamped_opt.has_value()) */
          /* /1* otherwise use the detection (if available) to orient the lurker and set its height //{ *1/ */
          /* { */
          /*   const auto ball_pos = ball_pose_stamped_opt.value().pose.block<3, 1>(0, 0); */
          /*   const double ball_dist = (ball_pos - cur_cmd_pos).norm(); */
          /*   ball_pos_opt = ball_pos; */
          /*   if (ball_dist > m_lurking_reaction_dist) */
          /*   { */
          /*     ROS_INFO_THROTTLE(1.0, "[LURKING]: Ball is too far (%.2fm > %.2fm), not reacting yet.", ball_dist, m_lurking_reaction_dist); */
          /*   } else */
          /*   { */
          /*     ROS_INFO_THROTTLE(1.0, "[LURKING]: Adapting yaw and height according to detection."); */
          /*     /1* const vec3_t dir_vec = (ball_pos - cur_cmd_pos).normalized(); *1/ */
          /*     /1* const double yaw = std::atan2(dir_vec.y(), dir_vec.x()); *1/ */
          /*     intercept_pos_opt = m_cur_lurk_pose; */
          /*     intercept_pos_opt.value().z() = ball_pos.z() + m_lurking_z_offset;  // set the height according to the detection height */
          /*   } */
          /* } */
          /* //} */
          if (!intercept_pos_opt.has_value())
          /* if neither prediction nor prediction is available, just stay at the current position //{ */
          {
            intercept_pos_opt = m_cur_lurk_pose + vec4_t(0, 0, m_lurking_z_offset, 0);
          }
          //}

          bool is_behind_yzplane = false;
          /* check for passthrough and transition to the next state if detected //{ */

          if (ball_pos_opt.has_value())
          {
            const auto ball_pos = ball_pos_opt.value();
            const double ball_dist = (ball_pos - cur_cmd_pos).norm();
            const double signed_ball_dist = signed_point_plane_distance(ball_pos, intersect_plane);
            ROS_WARN_THROTTLE(1.0, "[LURKING]: Signed ball distance from YZ plane: %.2fm!!", signed_ball_dist);
            if (signed_ball_dist < 0.0)
              is_behind_yzplane = true;
            if (ball_dist < m_lurking_passthrough_dist)
            {
              if (m_prev_signed_ball_dist_set && mrs_lib::signum(signed_ball_dist) != mrs_lib::signum(m_prev_signed_ball_dist))
              {
                ROS_ERROR("[LURKING]: BALL PASSTHROUGH DETECTED!!");
                if (m_lurking_land_check)
                  call_check_ball();
                if (m_lurking_land_after_first_passthrough)
                  land_there(m_landing_pose);
              }
              m_prev_signed_ball_dist = signed_ball_dist;
              m_prev_signed_ball_dist_set = true;
            }
          }

          //}

          // ignore the new setpoint if the ball is behind the YX plane
          /* publish the new command trajectory //{ */

          if (is_behind_yzplane)
            ROS_WARN("[LURKING]: The ball is behind the YZ plane - ignoring.");
          if (intercept_pos_opt.has_value() && !is_behind_yzplane)
          {
            auto lurking_pose = intercept_pos_opt.value();
            /* const auto lurking_point = lurking_pose.block<3, 1>(0, 0); */
            // limit the maximal reposition to m_lurking_max_reposition from the original lurk position
            const vec3_t orig_pose_offset = m_orig_lurk_pose.block<3, 1>(0, 0) + vec3_t(0, 0, m_lurking_z_offset);
            vec3_t reposition = lurking_pose.block<3, 1>(0, 0) - orig_pose_offset;
            double repos_dist = reposition.norm();
            if (repos_dist > m_lurking_max_reposition)
              lurking_pose.block<3, 1>(0, 0) = orig_pose_offset.block<3, 1>(0, 0) + reposition / repos_dist * m_lurking_max_reposition;
            reposition = lurking_pose.block<3, 1>(0, 0) - orig_pose_offset;
            repos_dist = reposition.norm();

            m_cur_lurk_pose_offset = lurking_pose;
            m_cur_lurk_pose = lurking_pose - vec4_t(0, 0, m_lurking_z_offset, 0);

            traj_t result_traj;
            add_point_to_trajectory(lurking_pose, result_traj);
            result_traj.header.frame_id = m_world_frame_id;
            result_traj.header.stamp = ros::Time(0);  // just fly now
            result_traj.header.stamp = cur_time;
            result_traj.use_heading = true;
            result_traj.fly_now = true;
            m_pub_cmd_traj.publish(result_traj);
            if (m_pub_dbg_traj.getNumSubscribers() > 0)
              m_pub_dbg_traj.publish(traj_to_path(result_traj, m_trajectory_sampling_dt));

            ROS_INFO_THROTTLE(1.0, "[LURKING]: Lurking at position [%.2f, %.2f, %.2f] (yaw: %.2f) - %.2fm from orig. lurk point", lurking_pose.x(),
                              lurking_pose.y(), lurking_pose.z(), lurking_pose.w(), repos_dist);
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

  /* delayed_init() method //{ */
  void BallPlanner::delayed_init([[maybe_unused]] const ros::TimerEvent& evt)
  {

    /* transform drop zone  //{ */

    {
      const auto tf_opt = get_transform_to_world(m_arena_frame_id, ros::Time::now());
      if (!tf_opt.has_value())
        return;

      const vec3_t tmp = tf_opt.value()*vec3_t(m_land_zone.x(), m_land_zone.y(), 0.0);
      m_landing_pose.block<3, 1>(0, 0) = tmp;
      m_landing_pose.z() = m_landing_height;
      m_landing_pose.w() = 0.0;
    }


    //}

    m_initialized = true;
    m_delayed_init_timer.stop();
  }
  //}

  // --------------------------------------------------------------
  // |                       Helper methods                       |
  // --------------------------------------------------------------

  /* pick_constraints() method //{ */
  /* std::string BallPlanner::pick_constraints(const double ball_dist) */
  /* { */
  /*   assert(!m_constraint_ranges.empty()); */
  /*   auto prev_constraint = std::begin(m_constraint_ranges)->second; */
  /*   for (const auto& keyval : m_constraint_ranges) */
  /*   { */
  /*     if (keyval.first > ball_dist) */
  /*       return prev_constraint; */
  /*     prev_constraint = keyval.second; */
  /*   } */
  /*   return prev_constraint; */
  /* } */

  std::string BallPlanner::pick_constraints(const state_t state)
  {
    const auto state_name = to_string(state);
    if (!m_constraint_states.count(state_name))
    {
      const auto ret = std::begin(m_constraint_states)->second;
      ROS_ERROR_THROTTLE(1.0, "[%s]: constraints for state '%s' were not specified! Using constraints '%s'.", m_node_name.c_str(), state_name.c_str(), ret.c_str());
      return ret;
    }
    const auto ret = m_constraint_states.at(state_name);
    return ret;
  }
  //}

  /* set_constraints() method //{ */
  void BallPlanner::set_constraints(const std::string& constraints_name)
  {
    mrs_msgs::String::Request req;
    mrs_msgs::String::Response res;
    req.value = constraints_name;
    if (!m_srv_set_constraints.call(req, res))
      /* ROS_INFO("Gain manager response: %s", res.message.c_str()); */
      /* else */
      ROS_ERROR_THROTTLE(1.0, "Failed to call service to set the constraints!");
  }
  //}

  /* reset_detector() method //{ */
  void BallPlanner::reset_detector()
  {
    std_srvs::Trigger::Request req;
    std_srvs::Trigger::Response res;
    if (m_srv_reset_detector.call(req, res))
      ROS_INFO("Detector response: %s", res.message.c_str());
    else
      ROS_ERROR("Failed to call service to reset the detector!");
  }
  //}

  /* reset_filter() method //{ */
  void BallPlanner::reset_filter()
  {
    std_srvs::Trigger::Request req;
    std_srvs::Trigger::Response res;
    if (m_srv_reset_filter.call(req, res))
      ROS_INFO("Filter response: %s", res.message.c_str());
    else
      ROS_ERROR("Failed to call service to reset the filter!");
  }
  //}

  /* land_there() method //{ */
  bool BallPlanner::land_there(const vec4_t& landing_pose)
  {
    mrs_msgs::ReferenceStampedSrv::Request req;
    req.header.frame_id = m_world_frame_id;
    req.header.stamp = ros::Time::now();
    req.reference.position.x = landing_pose.x();
    req.reference.position.y = landing_pose.y();
    req.reference.position.z = landing_pose.z();
    req.reference.heading = landing_pose.w();
    mrs_msgs::ReferenceStampedSrv::Response res;
    if (m_srv_land_there.call(req, res))
      ROS_INFO("Filter response: %s", res.message.c_str());
    else
      ROS_ERROR("Failed to call service to land there!");
    return res.success;
  }
  //}

  /* call_check_ball() method //{ */
  void BallPlanner::call_check_ball()
  {
    std_srvs::Trigger::Request req;
    std_srvs::Trigger::Response res;
    if (m_srv_ball_check.call(req, res))
      ROS_INFO("Ball fondler response: %s", res.message.c_str());
    else
      ROS_ERROR("Failed to call service to fondle the balls!");
  }
  //}

  /* get_yz_plane() method //{ */
  plane_t BallPlanner::get_yz_plane(const vec4_t& pose)
  {
    const vec3_t normal(cos(pose.w()), sin(pose.w()), 0.0);
    plane_t ret;
    ret.point = pose.block<3, 1>(0, 0);
    ret.normal = normal;
    return ret;
  }
  //}

  /* signed_point_plane_distance() method //{ */
  // returns +distance if the point is IN FRONT of the plane, -distance if the point is BEHIND the plane
  double BallPlanner::signed_point_plane_distance(const vec3_t& point, const plane_t& plane)
  {
    return (point - plane.point).dot(plane.normal.normalized());
  }
  //}

  /* line_plane_intersection_coeff() //{ */
  double line_plane_intersection_coeff(const vec3_t& line_start, const vec3_t& line_dir, const plane_t& plane)
  {
    const double den = line_dir.dot(plane.normal);
    const double nom = (plane.point - line_start).dot(plane.normal);
    // check for the case of a line parallel to the plane, the intersection would be at infinity
    // or anywhere in case of a line lying in the plane
    if (den == 0.0)
    {
      if (nom == 0.0)
        return 0.0;
      else
        return std::numeric_limits<double>::infinity();
    }
    return nom / den;
  }
  //}

  /* path_plane_intersection() method //{ */
  std::optional<vec3_t> BallPlanner::path_plane_intersection(const path_t& path, const plane_t& plane)
  {
    assert(path.poses.size() > 1);
    std::optional<vec3_t> intersection;
    vec3_t prev_end = to_eigen(path.poses.front().pose.position);
    for (size_t it = 1; it < path.poses.size(); it++)
    {
      const vec3_t line_start = prev_end;
      const vec3_t line_end = to_eigen(path.poses.at(it).pose.position);
      const vec3_t line_dir = line_end - line_start;
      const double cur_coeff = line_plane_intersection_coeff(line_start, line_dir, plane);
      if (cur_coeff >= 0.0 && cur_coeff <= 1.0)
      {
        intersection = line_start + cur_coeff * line_dir;
        break;  // we're only interested in the first intersection
      }
      prev_end = line_end;
    }
    return intersection;
  }
  //}

  /* yaw_from_quat() method //{ */
  double yaw_from_quat(const quat_t& quat)
  {
    const vec3_t unit_x_rotated = quat * vec3_t::UnitX();
    const double yaw = std::atan2(unit_x_rotated.y(), unit_x_rotated.x());
    return yaw;
  }
  //}

  /* load_dynparams() method //{ */
  void BallPlanner::load_dynparams(drcfg_t cfg)
  {
    m_lurking_min_observing_dur = ros::Duration(cfg.lurking__min_observing_duration);
    m_lurking_min_last_pts = cfg.lurking__min_last_points;
    m_lurking_z_offset = cfg.lurking__z_offset;
    m_lurking_max_reposition = cfg.lurking__max_reposition;
    /* m_lurking_max_reyaw = cfg.lurking__max_reyaw; */

    m_trajectory_horizon = cfg.trajectory__horizon;
    m_max_pts = std::floor(m_trajectory_horizon / m_trajectory_sampling_dt);
  }
  //}

  /* calc_offset_vector() method //{ */
  vec3_t BallPlanner::calc_horizontal_offset_vector(const vec3_t& dir_vec, const double tolerance)
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
  std::optional<geometry_msgs::TransformStamped> BallPlanner::get_transform_raw(const std::string& from_frame_id, const std::string& to_frame_id,
                                                                                   ros::Time stamp)
  {
    try
    {
      const ros::Duration timeout(1.0 / 100.0);
      geometry_msgs::TransformStamped transform = m_tf_buffer.lookupTransform(to_frame_id, from_frame_id, stamp, timeout);
      return transform;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("Error during transform from \"%s\" frame to \"%s\" frame.\n\tMSG: %s", from_frame_id.c_str(), to_frame_id.c_str(), ex.what());
      return std::nullopt;
    }
  }
  //}

  /* get_transform() method //{ */
  std::optional<Eigen::Affine3d> BallPlanner::get_transform(const std::string& from_frame_id, const std::string& to_frame_id, ros::Time stamp)
  {
    const auto tf_opt = get_transform_raw(from_frame_id, to_frame_id, stamp);
    if (tf_opt.has_value())
      return tf2::transformToEigen(tf_opt.value().transform);
    else
      return std::nullopt;
  }
  //}

  /* get_transform_to_world() method //{ */
  std::optional<Eigen::Affine3d> BallPlanner::get_transform_to_world(const std::string& frame_id, ros::Time stamp)
  {
    return get_transform(frame_id, m_world_frame_id, stamp);
  }
  //}

  /* process_detection() method //{ */
  std::optional<pose_stamped_t> BallPlanner::process_detection(const geometry_msgs::PoseWithCovarianceStamped& det)
  {
    const auto tf_opt = get_transform_to_world(det.header.frame_id, det.header.stamp);
    if (!tf_opt.has_value())
      return std::nullopt;
    const vec3_t pos = to_eigen(det.pose.pose.position);
    const vec3_t pos_global = tf_opt.value() * pos;

    const quat_t quat = to_eigen(det.pose.pose.orientation);
    const auto rot_global = tf_opt.value().rotation() * quat;
    const vec3_t x_vec = rot_global * vec3_t::UnitX();
    const double yaw = std::atan2(x_vec.y(), x_vec.x());
    const vec4_t pose_global = {pos_global.x(), pos_global.y(), pos_global.z(), yaw};

    return {{pose_global, det.header.stamp}};
  }
  //}

  /* process_detection() method //{ */
  std::optional<pose_stamped_t> BallPlanner::process_detection(const geometry_msgs::PoseStamped& det)
  {
    const auto tf_opt = get_transform_to_world(det.header.frame_id, det.header.stamp);
    if (!tf_opt.has_value())
      return std::nullopt;
    const vec3_t pos = to_eigen(det.pose.position);
    const vec3_t pos_global = tf_opt.value() * pos;

    const quat_t quat = to_eigen(det.pose.orientation);
    const auto rot_global = tf_opt.value().rotation() * quat;
    const vec3_t x_vec = rot_global * vec3_t::UnitX();
    const double yaw = std::atan2(x_vec.y(), x_vec.x());
    const vec4_t pose_global = {pos_global.x(), pos_global.y(), pos_global.z(), yaw};

    return {{pose_global, det.header.stamp}};
  }
  //}

  /* process_odom() method //{ */
  std::optional<vec4_t> BallPlanner::process_odom(const nav_msgs::Odometry& odom)
  {
    const auto tf_opt = get_transform_to_world(odom.header.frame_id, odom.header.stamp);
    if (!tf_opt.has_value())
      return std::nullopt;
    const vec3_t pos = to_eigen(odom.pose.pose.position);
    const quat_t quat = to_eigen(odom.pose.pose.orientation);
    const vec3_t pos_global = tf_opt.value() * pos;
    const quat_t quat_global = quat_t((tf_opt.value() * quat).rotation());
    const double yaw = yaw_from_quat(quat_global);
    const vec4_t pos_yaw(pos_global.x(), pos_global.y(), pos_global.z(), yaw);
    return pos_yaw;
  }
  //}

  /* get_ball_prediction() method //{ */
  std::optional<ball_filter::BallPrediction> BallPlanner::get_ball_prediction()
  {
    if (!m_sh_ball_prediction.newMsg())
      return std::nullopt;
    auto ret = *(m_sh_ball_prediction.getMsg());

    // check that the message is not too old
    const auto age = ros::Time::now() - ret.header.stamp;
    if (age > m_max_unseen_dur)
    {
      ROS_WARN_THROTTLE(1.0, "[BallPlanner]: Received prediction is too old, ignoring it (age: %.2f > %.2f)", age.toSec(), m_max_unseen_dur.toSec());
      return std::nullopt;
    }

    // try to transform it
    if (ret.header.frame_id != m_world_frame_id)
    {
      ROS_ERROR_THROTTLE(1.0, "[%s]: Cannot transform ball prediction message and it's not in the correct frame (expected: '%s', got '%s').",
                         m_node_name.c_str(), m_world_frame_id.c_str(), ret.header.frame_id.c_str());
      return std::nullopt;
    }
    return ret;
  }
  //}

  /* get_ball_position() method //{ */
  std::optional<pose_stamped_t> BallPlanner::get_ball_position()
  {
    if (!m_sh_ball_detection.hasMsg())
      return std::nullopt;
    const auto det = *(m_sh_ball_detection.getMsg());

    // check that the message is not too old
    const auto age = ros::Time::now() - det.header.stamp;
    if (age > m_max_unseen_dur)
    {
      ROS_WARN_THROTTLE(1.0, "[BallPlanner]: Received detection is too old, ignoring it (age: %.2f > %.2f)", age.toSec(), m_max_unseen_dur.toSec());
      return std::nullopt;
    }
    return process_detection(det);
  }
  //}

  /* get_ball_passthrough() method //{ */
  std::optional<pose_stamped_t> BallPlanner::get_ball_passthrough()
  {
    if (!m_sh_ball_passthrough.hasMsg())
      return std::nullopt;
    const auto det = *(m_sh_ball_passthrough.getMsg());
    return process_detection(det);
  }
  //}

  /* get_uav_position() method //{ */
  std::optional<vec4_t> BallPlanner::get_uav_position()
  {
    if (!m_sh_main_odom.hasMsg())
      return std::nullopt;
    const nav_msgs::Odometry odom = *(m_sh_main_odom.getMsg());
    return process_odom(odom);
  }
  //}

  /* get_uav_cmd_position() method //{ */
  std::optional<vec4_t> BallPlanner::get_uav_cmd_position()
  {
    if (!m_sh_cmd_odom.hasMsg())
      return std::nullopt;
    const nav_msgs::Odometry odom = *(m_sh_cmd_odom.getMsg());
    return process_odom(odom);
  }
  //}

  /* offset_path() method //{ */
  path_t BallPlanner::offset_path(const path_t& path, const vec3_t& off_vec)
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
    const double coeff = desired_time_diff.toSec() / time_diff.toSec();
    const quat_t interp_quat = quat1.slerp(coeff, quat2);
    return interp_quat;
  }
  //}

  /* trajectory_duration() method //{ */
  ros::Duration BallPlanner::trajectory_duration(const int n_traj_pts, const double dt)
  {
    const ros::Duration traj_dur(n_traj_pts > 0 ? (n_traj_pts - 1) * dt : 0);
    return traj_dur;
  }
  //}

  /* sample_trajectory_between_pts() method //{ */
  traj_t BallPlanner::sample_trajectory_between_pts(const vec3_t& from_pt, const vec3_t& to_pt, const double speed, const double dt, const double yaw,
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
  traj_t BallPlanner::sample_trajectory_from_path(const path_t& path, const double dt, const double speed, const size_t n_pts,
                                                     const std_msgs::Header& header)
  {
    assert(!path.poses.empty());
    traj_t ret;
    ret.points.reserve(n_pts);

    double dlen_remaining = dt * speed;
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
          ROS_WARN_STREAM_THROTTLE(1.0, "[BallPlanner]: Newest path point is older than current time, consider increasing the sampling horizon.");
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
      const vec3_t cur_pt = prev_pos + dlen_remaining * (next_pos - prev_pos) / dlen_cur;
      add_point_to_trajectory(cur_pt, ret);
      dlen_remaining += dt * speed;
    }

    ret.header = header;
    return ret;
  }
  //}

  /* sample_trajectory_from_path() method //{ */
  traj_t BallPlanner::sample_trajectory_from_path(const ros::Time& start_stamp, const path_t& path, const double dt, const size_t n_pts)
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
          ROS_WARN_STREAM_THROTTLE(1.0, "[BallPlanner]: Newest path point is older than current time, consider increasing the sampling horizon.");
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
  traj_t BallPlanner::join_trajectories(const traj_t& traj1, const traj_t& traj2)
  {
    traj_t ret;
    ret.header = traj1.header;
    ret.points.insert(std::end(ret.points), std::begin(traj1.points), std::end(traj1.points));
    ret.points.insert(std::end(ret.points), std::begin(traj2.points), std::end(traj2.points));
    return ret;
  }
  //}

  /* limit_cmd_vec_speed() method //{ */
  vec3_t BallPlanner::limit_cmd_vec_speed(const vec3_t& cmd_vector, const vec3_t& max_speed, const double dt, size_t max_pts)
  {
    assert(dt > 0.0);
    const vec3_t max_traj_dvec = max_speed * dt;
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
  traj_t BallPlanner::orient_trajectory_yaw(const traj_t& traj, const vec3_t& to_point)
  {
    traj_t ret = traj;
    ret.points.clear();
    const ros::Duration dt_dur(m_trajectory_sampling_dt);
    ros::Time cur_stamp = traj.header.stamp;
    for (auto& traj_pt : ret.points)
    {
      const vec3_t cur_traj_pt(traj_pt.position.x, traj_pt.position.y, traj_pt.position.z);
      const vec3_t offset_vec = to_point - cur_traj_pt;
      const double yaw = std::atan2(offset_vec.y(), offset_vec.x());
      traj_pt.heading = yaw;
      ret.points.push_back(traj_pt);
      cur_stamp += dt_dur;
    }
    return ret;
  }
  //}

  /* orient_trajectory_yaw_observe() method //{ */
  traj_t BallPlanner::orient_trajectory_yaw_observe(const traj_t& traj, const path_t& to_path)
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
          ROS_WARN_STREAM_THROTTLE(1.0, "[BallPlanner]: Newest path point is older than current time, consider increasing the sampling horizon.");
          break;
        }

        prev_pose_it++;
        prev_pose = to_path.poses.at(prev_pose_it);

        const auto next_pose_it = prev_pose_it + 1;
        next_pose = to_path.poses.at(next_pose_it);
      }

      auto cur_traj_pt_ros = traj.points.at(it);
      const vec3_t cur_traj_pt(cur_traj_pt_ros.position.x, cur_traj_pt_ros.position.y, cur_traj_pt_ros.position.z);
      const vec3_t cur_path_pt = linear_interpolation(cur_stamp, prev_pose, next_pose);
      const vec3_t offset_vec = cur_path_pt - cur_traj_pt;
      const double yaw = std::atan2(offset_vec.y(), offset_vec.x());
      cur_traj_pt_ros.heading = yaw;
      ret.points.push_back(cur_traj_pt_ros);
      cur_stamp += dt_dur;
    }
    return ret;
  }
  //}

  /* orient_trajectory_yaw_speed() method //{ */
  traj_t BallPlanner::orient_trajectory_yaw_speed(const traj_t& traj, const path_t& to_path)
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
          ROS_WARN_STREAM_THROTTLE(1.0, "[BallPlanner]: Newest path point is older than current time, consider increasing the sampling horizon.");
          break;
        }

        prev_pose_it++;
        prev_pose = to_path.poses.at(prev_pose_it);

        const auto next_pose_it = prev_pose_it + 1;
        next_pose = to_path.poses.at(next_pose_it);
      }

      auto cur_traj_pt_ros = traj.points.at(it);
      const quat_t cur_path_quat = linear_interpolation_quat(cur_stamp, prev_pose, next_pose);
      cur_traj_pt_ros.heading = yaw_from_quat(cur_path_quat);
      ret.points.push_back(cur_traj_pt_ros);
      cur_stamp += dt_dur;
    }
    return ret;
  }
  //}

  /* traj_to_path() method //{ */
  path_t BallPlanner::traj_to_path(const traj_t& traj, const double traj_dt)
  {
    path_t ret;
    ret.header = traj.header;

    const ros::Duration dt_dur(traj_dt);
    ros::Time cur_stamp = traj.header.stamp;
    for (const auto& tr_pt : traj.points)
    {
      const quat_t quat(Eigen::AngleAxisd(tr_pt.heading, vec3_t::UnitZ()));

      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = traj.header.frame_id;
      pose.header.stamp = cur_stamp;

      pose.pose.position.x = tr_pt.position.x;
      pose.pose.position.y = tr_pt.position.y;
      pose.pose.position.z = tr_pt.position.z;

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
  traj_t BallPlanner::path_to_traj(const path_t& path)
  {
    traj_t ret;
    ret.header = path.header;
    for (const auto& path_pose : path.poses)
    {
      traj_t::_points_type::value_type traj_pt;
      traj_pt.position.x = path_pose.pose.position.x;
      traj_pt.position.y = path_pose.pose.position.y;
      traj_pt.position.z = path_pose.pose.position.z;
      traj_pt.heading = yaw_from_quat(to_eigen(path_pose.pose.orientation));
      ret.points.push_back(traj_pt);
    }
    return ret;
  }
  //}

  /* point_to_traj() method //{ */
  traj_t BallPlanner::point_to_traj(const vec3_t& point, const size_t n_pts)
  {
    traj_t ret;
    ret.points.reserve(n_pts);
    mrs_msgs::Reference tr_point;
    tr_point.position.x = point.x();
    tr_point.position.y = point.y();
    tr_point.position.z = point.z();
    for (size_t it = 0; it < n_pts; it++)
      ret.points.push_back(tr_point);
    return ret;
  }
  //}

  // | ------------------ Visualization methods ----------------- |
  /* methods //{ */

  /* to_output_message() method //{ */
  sensor_msgs::PointCloud2 BallPlanner::to_output_message(const std::vector<pose_stamped_t>& points, const std_msgs::Header& header)
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
        *iter_x = points.at(it).pose.x();
        *iter_y = points.at(it).pose.y();
        *iter_z = points.at(it).pose.z();
      }
    }

    return ret;
  }
  //}

  /* to_output_message() method //{ */
  geometry_msgs::PoseStamped BallPlanner::to_output_message(const vec4_t& position, const std_msgs::Header& header)
  {
    geometry_msgs::PoseStamped ret;
    ret.header = header;

    ret.pose.position.x = position.x();
    ret.pose.position.y = position.y();
    ret.pose.position.z = position.z();

    const double yaw = position.w();
    const vec3_t heading_vec(std::cos(yaw), std::sin(yaw), 0);
    const quat_t quat = mrs_lib::geometry::quaternionBetween(vec3_t(1.0, 0.0, 0.0), heading_vec);

    ret.pose.orientation.x = quat.x();
    ret.pose.orientation.y = quat.y();
    ret.pose.orientation.z = quat.z();
    ret.pose.orientation.w = quat.w();

    return ret;
  }
  //}

  /* to_output_message() method //{ */
  visualization_msgs::Marker BallPlanner::to_output_message(const cv::Vec6f& line, const std_msgs::Header& header)
  {
    visualization_msgs::Marker ret;
    ret.header = header;
    ret.color.a = 1.0;
    ret.color.r = 1.0;
    ret.scale.x = 0.1;
    ret.type = visualization_msgs::Marker::LINE_STRIP;
    ret.pose.orientation.w = 1.0;

    geometry_msgs::Point pt1;
    pt1.x = line(3) - 50 * line(0);
    pt1.y = line(4) - 50 * line(1);
    pt1.z = line(5) - 50 * line(2);
    geometry_msgs::Point pt2;
    pt2.x = pt1.x + 100 * line(0);
    pt2.y = pt1.y + 100 * line(1);
    pt2.z = pt1.z + 100 * line(2);

    ret.points.push_back(pt1);
    ret.points.push_back(pt2);

    return ret;
  }
  //}

  /* to_output_message() method //{ */
  sensor_msgs::PointCloud2 BallPlanner::to_output_message(const std::vector<cv::Point3d>& points, const std_msgs::Header& header)
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
        *iter_x = points.at(it).x;
        *iter_y = points.at(it).y;
        *iter_z = points.at(it).z;
      }
    }

    return ret;
  }
  //}

  /* to_output_message() method //{ */
  geometry_msgs::PointStamped BallPlanner::to_msg(const vec3_t& point, const std_msgs::Header& header)
  {
    geometry_msgs::PointStamped ret;
    ret.header = header;
    ret.point.x = point.x();
    ret.point.y = point.y();
    ret.point.z = point.z();
    return ret;
  }
  //}

  /* plane_visualization //{ */
  visualization_msgs::MarkerArray BallPlanner::plane_visualization(const plane_t& plane, const std_msgs::Header& header)
  {
    visualization_msgs::MarkerArray ret;

    const auto pos = plane.point;
    const auto quat = quat_t::FromTwoVectors(vec3_t::UnitZ(), plane.normal);

    const double size = 20;
    geometry_msgs::Point ptA;
    ptA.x = size;
    ptA.y = size;
    ptA.z = 0;
    geometry_msgs::Point ptB;
    ptB.x = -size;
    ptB.y = size;
    ptB.z = 0;
    geometry_msgs::Point ptC;
    ptC.x = -size;
    ptC.y = -size;
    ptC.z = 0;
    geometry_msgs::Point ptD;
    ptD.x = size;
    ptD.y = -size;
    ptD.z = 0;

    /* borders marker //{ */
    {
      visualization_msgs::Marker borders_marker;
      borders_marker.header = header;

      borders_marker.ns = "borders";
      borders_marker.id = 0;
      borders_marker.type = visualization_msgs::Marker::LINE_LIST;
      borders_marker.action = visualization_msgs::Marker::ADD;

      borders_marker.pose.position.x = pos.x();
      borders_marker.pose.position.y = pos.y();
      borders_marker.pose.position.z = pos.z();

      borders_marker.pose.orientation.x = quat.x();
      borders_marker.pose.orientation.y = quat.y();
      borders_marker.pose.orientation.z = quat.z();
      borders_marker.pose.orientation.w = quat.w();

      borders_marker.scale.x = 0.03;

      borders_marker.color.a = 0.4;  // Don't forget to set the alpha!
      borders_marker.color.r = 0.0;
      borders_marker.color.g = 1.0;
      borders_marker.color.b = 0.0;

      borders_marker.points.push_back(ptA);
      borders_marker.points.push_back(ptB);

      borders_marker.points.push_back(ptB);
      borders_marker.points.push_back(ptC);

      borders_marker.points.push_back(ptC);
      borders_marker.points.push_back(ptD);

      borders_marker.points.push_back(ptD);
      borders_marker.points.push_back(ptA);

      ret.markers.push_back(borders_marker);
    }
    //}

    /* plane marker //{ */
    {
      visualization_msgs::Marker plane_marker;
      plane_marker.header = header;

      plane_marker.ns = "plane";
      plane_marker.id = 1;
      plane_marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
      plane_marker.action = visualization_msgs::Marker::ADD;

      plane_marker.pose.position.x = pos.x();
      plane_marker.pose.position.y = pos.y();
      plane_marker.pose.position.z = pos.z();

      plane_marker.pose.orientation.x = quat.x();
      plane_marker.pose.orientation.y = quat.y();
      plane_marker.pose.orientation.z = quat.z();
      plane_marker.pose.orientation.w = quat.w();

      plane_marker.scale.x = 1;
      plane_marker.scale.y = 1;
      plane_marker.scale.z = 1;

      plane_marker.color.a = 0.1;  // Don't forget to set the alpha!
      plane_marker.color.r = 0.0;
      plane_marker.color.g = 1.0;
      plane_marker.color.b = 0.0;

      // triangle ABC
      plane_marker.points.push_back(ptA);
      plane_marker.points.push_back(ptB);
      plane_marker.points.push_back(ptC);

      // triangle ACD
      plane_marker.points.push_back(ptA);
      plane_marker.points.push_back(ptC);
      plane_marker.points.push_back(ptD);
      ret.markers.push_back(plane_marker);
    }
    //}

    return ret;
  }
  //}

  //}

  // | -------------------------- OTHER ------------------------- |
  /* onInit() //{ */

  void BallPlanner::onInit()
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

    const double planning_period = pl.loadParam2<double>("planning_period");

    pl.loadParam("world_frame_id", m_world_frame_id);
    pl.loadParam("arena_frame_id", m_arena_frame_id);
    pl.loadParam("max_unseen_duration", m_max_unseen_dur);

    pl.loadParam("trajectory/sampling_dt", m_trajectory_sampling_dt);
    pl.loadParam("trajectory/target_reached_distance", m_trajectory_tgt_reached_dist);
    pl.loadParam("trajectory/default_speed", m_trajectory_default_speed);

    pl.loadParam("pogo/min_height", m_pogo_min_height);
    pl.loadParam("pogo/max_height", m_pogo_max_height);
    pl.loadParam("pogo/speed", m_pogo_speed);

    /* pl.loadParam("lurking/observe_dist", m_lurking_observe_dist); */
    pl.loadParam("lurking/reaction_dist", m_lurking_reaction_dist);
    pl.loadParam("lurking/min_last_points", m_lurking_min_last_pts);
    pl.loadParam("lurking/min_last_duration", m_lurking_min_last_dur);
    pl.loadParam("lurking/passthrough_detection_distance", m_lurking_passthrough_dist);
    pl.loadParam("lurking/land_check", m_lurking_land_check);
    pl.loadParam("lurking/land_after_first_passthrough", m_lurking_land_after_first_passthrough);

    pl.loadParam("constraint_states", m_constraint_states);

    /* load and print constraint ranges //{ */

    /* const auto constraint_ranges_unsorted = pl.loadParam2<std::map<std::string, double>>("constraint_ranges"); */
    /* for (const auto& keyval : constraint_ranges_unsorted) */
    /*   m_constraint_ranges.emplace(keyval.second, keyval.first); */
    /* ROS_INFO("[%s]: Sorted constraint ranges:", m_node_name.c_str()); */
    /* std::cout << "{" <<std::endl; */
    /* double prev_dist = 0.0; */
    /* for (const auto& keyval : m_constraint_ranges) */
    /* { */
    /*   std::cout << "\t" << prev_dist << "~" << keyval.first << ":\t" << keyval.second << std::endl; */
    /*   prev_dist = keyval.first; */
    /* } */
    /* std::cout << "}" <<std::endl; */

    //}

    pl.loadMatrixStatic<2, 1>("dropoff_center", m_land_zone);
    pl.loadParam("landing_height", m_landing_height);
    pl.loadMatrixStatic<4, 1>("start_position", m_start_pose);

    if (!pl.loadedSuccessfully())
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
    mrs_lib::SubscribeHandlerOptions shopts;
    shopts.nh = nh;
    shopts.node_name = m_node_name;
    shopts.no_message_timeout = ros::Duration(5.0);

    mrs_lib::construct_object(m_sh_ball_detection, shopts, "ball_detection");
    mrs_lib::construct_object(m_sh_ball_passthrough, shopts, "ball_passthrough");
    mrs_lib::construct_object(m_sh_ball_prediction, shopts, "ball_prediction");
    mrs_lib::construct_object(m_sh_cmd_odom, shopts, "cmd_odom");
    mrs_lib::construct_object(m_sh_main_odom, shopts, "main_odom");
    mrs_lib::construct_object(m_sh_tracker_diags, shopts, "tracker_diagnostics");

    m_srv_start = nh.advertiseService("start_state_machine", &BallPlanner::start_callback, this);
    m_srv_stop = nh.advertiseService("stop_state_machine", &BallPlanner::stop_callback, this);
    m_srv_land = nh.advertiseService("land", &BallPlanner::land_callback, this);

    //}

    /* publishers //{ */

    m_pub_cmd_traj = nh.advertise<traj_t>("commanded_trajectory", 1);
    m_pub_dbg_traj = nh.advertise<nav_msgs::Path>("debug_trajectory", 1);
    m_pub_dbg_ball_positions = nh.advertise<sensor_msgs::PointCloud2>("ball_positions", 1);
    m_pub_dbg_lurking_points = nh.advertise<sensor_msgs::PointCloud2>("lurking_points", 1, true);
    m_pub_dbg_lurking_position = nh.advertise<geometry_msgs::PoseStamped>("lurking_position", 1, true);
    m_pub_dbg_xy_plane = nh.advertise<visualization_msgs::MarkerArray>("yz_plane", 1);
    m_pub_dbg_int_pt = nh.advertise<geometry_msgs::PointStamped>("intersection_point", 1);
    m_pub_dbg_linefit = nh.advertise<visualization_msgs::Marker>("line_fit", 1, true);
    m_pub_dbg_linefit_pts = nh.advertise<sensor_msgs::PointCloud2>("line_fit_pts", 1, true);

    //}

    /* services //{ */

    m_srv_reset_detector = nh.serviceClient<std_srvs::Trigger>("reset_detector");
    m_srv_reset_filter = nh.serviceClient<std_srvs::Trigger>("reset_ball_filter");
    m_srv_set_constraints = nh.serviceClient<mrs_msgs::String>("set_constraints");
    m_srv_land_there = nh.serviceClient<mrs_msgs::ReferenceStampedSrv>("land_there");
    m_srv_ball_check = nh.serviceClient<std_srvs::Trigger>("ball_check");

    //}

    /* timers  //{ */

    m_main_loop_timer = nh.createTimer(ros::Duration(planning_period), &BallPlanner::main_loop, this);
    m_delayed_init_timer = nh.createTimer(ros::Duration(1.0), &BallPlanner::delayed_init, this);

    //}

    m_ball_positions.reserve(200);
    m_state = state_enum::going_to_nextpos_low;

    m_pogo_height_range = m_pogo_max_height - m_pogo_min_height;

    ROS_INFO("[%s]: initialized", m_node_name.c_str());
  }

  //}

  /* BallPlanner::start_callback() method //{ */

  bool BallPlanner::start_callback(mrs_msgs::SetInt::Request& req, mrs_msgs::SetInt::Response& resp)
  {
    if (m_activated)
      resp.message = "State machine already active! Got '" + std::to_string(req.value) + "'.";
    else
      resp.message = "Activated state machine. Got '" + std::to_string(req.value) + "'.";
    switch (req.value)
    {
      case 0: m_strat = strat_t::lurk_arc_endpose; break;
      case 1: m_strat = strat_t::lurk_most_probable; break;
      case 2: m_strat = strat_t::lurk_lowest; break;
      default: m_strat = strat_t::unknown; break;
    }
    resp.message += " Selected strategy: '" + to_string(m_strat) + "'.";
    m_activated = true;
    resp.success = true;
    return true;
  }

  //}

  /* BallPlanner::stop_callback() method //{ */

  bool BallPlanner::stop_callback([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
  {
    if (!m_activated)
      resp.message = "State machine is already inactive!";
    else
      resp.message = "Deactivated state machine.";
    m_activated = false;
    resp.success = true;
    m_state = state_enum::going_to_nextpos_low;
    return true;
  }

  //}

  /* BallPlanner::land_callback() method //{ */

  bool BallPlanner::land_callback([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
  {
    resp.message = "LANDING IN SATAN'S NAME!";
    m_activated = false;
    m_land = true;
    resp.success = true;
    return true;
  }

  //}

}  // namespace ball_planner

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ball_planner::BallPlanner, nodelet::Nodelet)
