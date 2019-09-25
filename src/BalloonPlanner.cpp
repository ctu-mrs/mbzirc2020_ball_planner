#include <balloon_planner/BalloonPlanner.h>

namespace balloon_planner
{

  /* main_loop() method //{ */
  void BalloonPlanner::main_loop([[maybe_unused]] const ros::TimerEvent& evt)
  {
    load_dynparams(m_drmgr_ptr->config);

    auto cur_pos_res = get_current_position();
    if (m_sh_predicted_path->new_data() && m_sh_fitted_plane->has_data() && cur_pos_res.has_value())
    {
      const vec3_t cur_pos = cur_pos_res.value();
      const auto pred_path = *(m_sh_predicted_path->get_data());
      const auto plane_params = *(m_sh_fitted_plane->get_data());
      const auto cur_t = ros::Time::now();
      if (!pred_path.poses.empty())
      {
        ROS_ERROR_STREAM_THROTTLE(1.0, "[BalloonPlanner]: Received empty predicted path, cannot plan trajectory!");
        return;
      }
      if (pred_path.poses.at(pred_path.poses.size()-1).header.stamp < cur_t)
      {
        ROS_ERROR_STREAM_THROTTLE(1.0, "[BalloonPlanner]: Newest predicted pose is older that current time, cannot plan trajectory!");
        return;
      }

      const auto offset_vector = calc_path_offset_vector(plane_params);
      const auto tgt_path = offset_path(pred_path, offset_vector, m_path_offset);

      const auto approach_res = find_approach_pt(cur_pos, cur_t, tgt_path, m_approach_speed);
      if (!approach_res.has_value())
      {
        ROS_ERROR_STREAM_THROTTLE(1.0, "[BalloonPlanner]: Failed to find a viable approach point to the predicted trajectory!");
        return;
      }
      const auto [approach_pt, approach_stamp] = approach_res.value();
      const auto approach_traj = sample_trajectory_between_pts(cur_pos, approach_pt, m_approach_speed, m_trajectory_sampling_dt);

      const auto approach_duration = approach_stamp - cur_t;
      const auto remaining_traj_dur = ros::Duration(m_trajectory_horizon) - approach_duration;
      std::optional<traj_t> follow_traj_res = std::nullopt;
      if (remaining_traj_dur > ros::Duration(0.0))
      {
        follow_traj_res = sample_trajectory_from_path(approach_stamp, tgt_path, remaining_traj_dur, m_trajectory_sampling_dt);
        ROS_INFO_STREAM_THROTTLE(1.0, "[BalloonPlanner]: Approach traj:" << approach_duration.toSec() << "s, " << approach_traj.points.size() << "pts; follow traj: " << remaining_traj_dur.toSec() << "s, " << follow_traj_res.value_or(traj_t()).points.size());
      } else
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "[BalloonPlanner]: Approach trajectory takes longer than trajectory horizon (" << approach_duration.toSec() << "s > " << m_trajectory_horizon << "s).");
      }
      const auto joined_traj = join_trajectories(approach_traj, follow_traj_res.value_or(traj_t()));
      traj_t result_traj = orient_trajectory_yaw(joined_traj, pred_path);
      result_traj.header.frame_id = m_world_frame;
      result_traj.header.stamp = cur_t;
      result_traj.use_yaw = true;
      assert(result_traj.points.size() == m_max_pts);

      m_pub_cmd_traj.publish(result_traj);
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

  /* get_current_position() method //{ */
  std::optional<vec3_t> BalloonPlanner::get_current_position()
  {
    Eigen::Affine3d uav2world_tf;
    const bool success = get_transform_to_world(m_uav_frame_id, ros::Time::now(), uav2world_tf);
    if (success)
      return uav2world_tf * vec3_t(0, 0, 0);
    else
      return std::nullopt;
  }
  //}

  /* calc_path_offset_vector() method //{ */
  vec3_t BalloonPlanner::calc_path_offset_vector(const plane_t& plane_params, const double tolerance)
  {
    const vec3_t normal(plane_params.normal.x, plane_params.normal.y, plane_params.normal.z);
    vec3_t ret(normal.x(), normal.y(), 0.0);
    if (ret.norm() < tolerance)
      ret = vec3_t(-1.0, 0.0, 0.0); // if the plane is horizontal, offset in x by default
    else
      ret = ret.normalized();
    return ret;
  }
  //}

  /* offset_path() method //{ */
  path_t BalloonPlanner::offset_path(const path_t& path, const vec3_t& vector, const double offset)
  {
    const vec3_t off_vec = vector*offset;
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
    const vec3_t line_dir = (line_pt2 - line_pt1).normalized();
    const vec3_t diff_vec = to_pt - line_pt1;
    const double param = line_dir.dot(diff_vec);
    return param;
  }
  //}

  /* linear_interpolation() method //{ */
  vec3_t linear_interpolation(const ros::Time& to_time, const geometry_msgs::PoseStamped& line_pt1, const geometry_msgs::PoseStamped& line_pt2)
  {
    const ros::Duration time_diff = line_pt2.header.stamp - line_pt1.header.stamp;
    assert(!time_diff.isZero());

    const vec3_t pt1(line_pt1.pose.position.x, line_pt1.pose.position.y, line_pt1.pose.position.z);
    const vec3_t pt2(line_pt2.pose.position.x, line_pt2.pose.position.y, line_pt2.pose.position.z);
    const vec3_t pt_diff = pt2 - pt1;
    const ros::Duration desired_time_diff = to_time - line_pt1.header.stamp;
    const vec3_t interp_pt = pt1 + pt_diff/time_diff.toSec()*desired_time_diff.toSec();
    return interp_pt;
  }
  //}

  /* find_approach_pt() method //{ */
  std::optional<std::tuple<vec3_t, ros::Time>> BalloonPlanner::find_approach_pt(const vec3_t& from_pt, const ros::Time& from_time, const path_t& to_path, const double speed)
  {
    vec3_t closest_pt;
    ros::Duration closest_dur;
    bool closest_dur_set = false;
  
    geometry_msgs::PoseStamped prev_pose;
    bool prev_pose_set = false;
    for (const auto& pose : to_path.poses)
    {
      if (prev_pose_set)
      {
        const vec3_t line_pt1(prev_pose.pose.position.x, prev_pose.pose.position.y, prev_pose.pose.position.z);
        const vec3_t line_pt2(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        const auto cur_param = find_closest_line_param(from_pt, line_pt1, line_pt2);
        const ros::Time t1 = prev_pose.header.stamp;
        const ros::Time t2 = pose.header.stamp;
        const auto time_offset = from_time - t1;
        const ros::Duration cur_dur = (t2 - t1)*cur_param - time_offset;
  
        if (cur_dur > ros::Duration(0) && (!closest_dur_set || cur_dur < closest_dur))
        {
          closest_pt = (line_pt2 - line_pt1)*cur_param;
          closest_dur = cur_dur;
        }
      }
      prev_pose = pose;
    }
  
    if (!closest_dur_set)
      return std::nullopt;
  
    const auto scaled_dur = closest_dur*(1.0/speed);
    const ros::Time closest_time = from_time + scaled_dur;
    return {{closest_pt, closest_time}};
  }
  //}

  /* sample_trajectory_between_pts() method //{ */
  traj_t BalloonPlanner::sample_trajectory_between_pts(const vec3_t& from_pt, const vec3_t& to_pt, const double speed, const double dt)
  {
    assert(speed > 0.0);
    assert(dt > 0.0);
  
    const vec3_t diff_vec = to_pt - from_pt;
    const double dist = diff_vec.norm();
    const size_t n_pts = std::floor(dist/speed);
    const vec3_t d_vec = speed*dt*diff_vec/dist;
  
    traj_t ret;
    ret.points.reserve(n_pts);
    for (size_t it = 0; it < n_pts; it++)
    {
      const auto cur_pt = from_pt + it*d_vec;
      mrs_msgs::TrackerPoint tr_pt;
      tr_pt.x = cur_pt.x();
      tr_pt.y = cur_pt.y();
      tr_pt.z = cur_pt.z();
    }
    return ret;
  }
  //}

  /* sample_trajectory_from_path() method //{ */
  traj_t BalloonPlanner::sample_trajectory_from_path(const ros::Time& start_stamp, const path_t& path, const ros::Duration& dur, const double dt)
  {
    assert(!path.poses.empty());
    const size_t n_pts = std::min(m_max_pts, size_t(std::floor(dur.toSec() / dt)));
    const ros::Duration dt_dur(dt);
  
    traj_t ret;
    ret.points.reserve(n_pts);
  
    ros::Time cur_time = start_stamp;
    int prev_pose_it = -1;
    for (size_t it = 0; it < n_pts; it++)
    {
      geometry_msgs::PoseStamped prev_pose, next_pose;
      // Find prev_pose which is before start_stamp with cur_pose being after start_stamp and next after prev_pose
      do
      {
        if (prev_pose_it >= (int)path.poses.size()-1)
        {
          ROS_ERROR_STREAM_THROTTLE(1.0, "[BalloonPlanner]: Newest path point is older than current time, cannot sample trajectory from path!");
          break;
        }
  
        prev_pose_it++;
        prev_pose = path.poses.at(prev_pose_it);
  
        const auto next_pose_it = prev_pose_it+1;
        next_pose = path.poses.at(next_pose_it);
  
      } while (next_pose.header.stamp < cur_time);
  
      const auto cur_pt = linear_interpolation(cur_time, prev_pose, next_pose);
      mrs_msgs::TrackerPoint tr_pt;
      tr_pt.x = cur_pt.x();
      tr_pt.y = cur_pt.y();
      tr_pt.z = cur_pt.z();
      ret.points.push_back(tr_pt);
  
      cur_time += dt_dur;
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

  traj_t BalloonPlanner::orient_trajectory_yaw(const traj_t& traj, const path_t& to_path)
  {
    return traj; // TODO
  }

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
  pl.load_param("path_offset", m_path_offset);
  pl.load_param("approach_speed", m_approach_speed);

  pl.load_param("trajectory/sampling_dt", m_trajectory_sampling_dt);
  pl.load_param("trajectory/horizon", m_trajectory_horizon);

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
  m_sh_predicted_path = smgr.create_handler<nav_msgs::Path, time_consistent>("ball_predicted_path", ros::Duration(5.0));
  m_sh_fitted_plane = smgr.create_handler<balloon_filter::Plane, time_consistent>("ball_fitted_plane", ros::Duration(5.0));

  //}

  /* publishers //{ */

  m_pub_cmd_traj = nh.advertise<mrs_msgs::TrackerTrajectory>("commanded_trajectory", 1);

  //}

  /* profiler //{ */

  m_profiler_ptr = std::make_unique<mrs_lib::Profiler>(nh, m_node_name, false);

  //}

  /* timers  //{ */

  m_main_loop_timer = nh.createTimer(ros::Duration(planning_period), &BalloonPlanner::main_loop, this);

  //}

  m_max_pts = std::floor(m_trajectory_horizon/m_trajectory_sampling_dt);

  ROS_INFO("[%s]: initialized", m_node_name.c_str());
}

//}

}  // namespace balloon_planner

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(balloon_planner::BalloonPlanner, nodelet::Nodelet)
