#include "BalloonPlanner.h"

namespace balloon_planner
{

/* onInit() //{ */

void BalloonPlanner::onInit() {

  ROS_INFO("[%s]: Initializing", m_node_name.c_str());
  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
  ros::Time::waitForValid();

  /* initialize variables //{ */


  //}

  /* load parameters //{ */

  mrs_lib::ParamLoader param_loader(nh, m_node_name);

  /* param_loader.load_param("", ); */

  //}

  /* timers  //{ */


  //}

  /* subscribers //{ */

  mrs_lib::SubscribeMgr smgr(nh);
  m_sh_balloons = smgr.create_handler_threadsafe<sensor_msgs::PointCloud>("balloon_cloud_in", 10, ros::TransportHints().tcpNoDelay(), ros::Duration(5.0));

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
