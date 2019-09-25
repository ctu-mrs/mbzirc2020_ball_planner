#include <balloon_planner/BalloonPlanner.h>

namespace balloon_planner
{

  /* main_loop() method //{ */
  void BalloonPlanner::main_loop([[maybe_unused]] const ros::TimerEvent& evt)
  {
    load_dynparams(m_drmgr_ptr->config);

    if (m_sh_predicted_path->new_data())
    {
      const auto pred_path = *(m_sh_predicted_path->get_data());
  
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
