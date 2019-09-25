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

      const auto offset_vector = calc_path_offset_vector(plane_params);
      const auto tgt_path = offset_path(pred_path, offset_vector, m_path_offset);

      const auto approach_res = find_approach_pt(cur_pos, cur_t, tgt_path, m_approach_speed);
      if (!approach_res.has_value())
      {
        ROS_ERROR_STREAM_THROTTLE(1.0, "[BalloonPlanner]: Failed to find a viable approach point to the predicted trajectory!");
        return;
      }
      const auto [approach_pt, approach_stamp] = approach_res.value();
      const auto approach_duration = approach_stamp - cur_t;
      const auto remaining_traj_dur = ros::Duration(m_trajectory_horizon) - approach_duration;
      const auto approach_traj = sample_trajectory_between_pts(cur_pos, approach_pt, m_approach_speed, m_trajectory_sampling_dt);
      const auto follow_traj = sample_trajectory_from_path(approach_stamp, tgt_path, remaining_traj_dur, m_trajectory_sampling_dt);
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

  double find_closest_line_param(const vec3_t& to_pt, const vec3_t line_pt1, const vec3_t line_pt2)
  {
    const vec3_t line_dir = (line_pt2 - line_pt1).normalized();
    const vec3_t diff_vec = to_pt - line_pt1;
    const double param = line_dir.dot(diff_vec);
    return param;
  }

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
    /* return std::optional<std::tuple<vec3_t, ros::Time>>{closest_pt, closest_time}; */
    return {{closest_pt, closest_time}};
  }

  /* BalloonPlanner::traj_t BalloonPlanner::sample_trajectory_between_pts(const vec3_t& from_pt, const vec3_t& to_pt, const double speed, const double dt); */
  /* BalloonPlanner::traj_t BalloonPlanner::sample_trajectory_from_path(const ros::Time& start_stamp, const path_t& path, const ros::Duration& dur, const double dt); */

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

  ROS_INFO("[%s]: initialized", m_node_name.c_str());
}

//}

}  // namespace balloon_planner

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(balloon_planner::BalloonPlanner, nodelet::Nodelet)
