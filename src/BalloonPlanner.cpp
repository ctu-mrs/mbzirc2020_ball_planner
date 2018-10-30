#include "BalloonPlanner.h"

namespace balloon_planner
{

void BalloonPlanner::main_loop(const ros::TimerEvent& evt)
{

  if (m_sh_balloons->new_data())
  {
    ROS_INFO("[%s]: Processing new data", m_node_name.c_str());

    sensor_msgs::PointCloud balloons = m_sh_balloons->get_data();

    ROS_INFO("[%s]: New data processed", m_node_name.c_str());
  }

}

/* onInit() //{ */

void BalloonPlanner::onInit()
{

  ROS_INFO("[%s]: Initializing", m_node_name.c_str());
  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
  ros::Time::waitForValid();

  /* initialize variables //{ */

  m_last_lkf_id = 0;

  //}

  /* load parameters //{ */

  mrs_lib::ParamLoader pl(nh, m_node_name);

  double planning_period = pl.load_param2<double>("planning_period");

  if (!pl.loaded_successfully())
  {
    ROS_ERROR("Some compulsory parameters were not loaded successfully, ending the node");
    ros::shutdown();
  }

  // LOAD DYNAMIC PARAMETERS
  m_drmgr_ptr = std::make_unique<drmgr_t>(nh, m_node_name);
  if (!m_drmgr_ptr->loaded_successfully())
  {
    ROS_ERROR("Some dynamic parameter default values were not loaded successfully, ending the node");
    ros::shutdown();
  }

  //}

  /* timers  //{ */

  m_main_loop_timer = nh.createTimer(ros::Duration(planning_period), &BalloonPlanner::main_loop, this);
  m_lkf_update_timer = nh.createTimer(ros::Duration(m_lkf_dt), &BalloonPlanner::lkf_update, this);

  //}

  /* subscribers //{ */

  mrs_lib::SubscribeMgr smgr(nh);
  m_sh_balloons = smgr.create_handler_threadsafe<sensor_msgs::PointCloud>("balloon_cloud_in", 10, ros::TransportHints().tcpNoDelay(), ros::Duration(5.0));

  if (!smgr.loaded_successfully())
  {
    ROS_ERROR("Unable to subscribe to some topics, ending the node");
    ros::shutdown();
  }

  //}

  /* publishers //{ */

  m_pub_odom_balloon = nh.advertise<nav_msgs::Odometry>("balloon_odom_out", 1);

  //}

  /* profiler //{ */

  m_profiler_ptr = std::make_unique<mrs_lib::Profiler>(nh, m_node_name, false);

  //}

  m_is_initialized = true;

  ROS_INFO("[%s]: initialized", m_node_name.c_str());
}

//}

/* /1* callbackBalloonCloud() //{ *1/ */

/* void BalloonPlanner::callbackBalloonCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& msg) { */

/*   if (!m_is_initialized) */
/*     return; */

/*   ROS_DEBUG_THROTTLE(1.0, "[%s]: Obtained balloon cloud", m_node_name); */

/*   mrs_lib::Routine profiler_routine = m_profiler->createRoutine("callbackBalloonCloud"); */

/*   ROS_INFO("[%s]: balloon cloud size: %lu", m_node_name, msg->size()); */

/* } */

/* //} */

/* /1* //{ callbackReconfigure() *1/ */

/* /1* void BalloonPlanner::callbackReconfigure([[maybe_unused]] balloon_planner::bpConfig& config, [[maybe_unused]] uint32_t level) { *1/ */

/* /1*   if (!m_is_initialized) *1/ */
/* /1*     return; *1/ */

/* /1*   ROS_INFO( *1/ */
/* /1*       "Reconfigure Request: " *1/ */
/* /1*       "median_blur: %d, scale: %f, range_min: %f, range_max: %f", *1/ */
/* /1*       config.median_blur, config.scale, config.range_min, config.range_max); *1/ */

/* /1*   _median_blur = config.median_blur; *1/ */
/* /1*   _scale       = config.scale; *1/ */
/* /1*   _range_min   = config.range_min; *1/ */
/* /1*   _range_max   = config.range_max; *1/ */
/* /1* } *1/ */
/* //} */

}  // namespace balloon_planner

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(balloon_planner::BalloonPlanner, nodelet::Nodelet)
