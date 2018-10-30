#include "BalloonPlanner.h"

namespace balloon_planner
{

/* onInit() //{ */

void BalloonPlanner::onInit() {

  ROS_INFO("[BalloonPlanner]: Initializing");
  ros::NodeHandle                 nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();
  ros::Time::waitForValid();

  /* initialize variables //{ */


  //}

  /* load parameters //{ */

  mrs_lib::ParamLoader param_loader(nh_, "BalloonPlanner");

  /* param_loader.load_param("", ); */

  //}

  /* timers  //{ */


  //}

  /* subscribers //{ */

  m_sub_cloud_balloon_ = nh_.subscribe("balloon_cloud_in", 10, &BalloonPlanner::callbackBalloonCloud, this, ros::TransportHints().tcpNoDelay());

  //}

  /* publishers //{ */

  m_pub_odom_balloon_ = nh_.advertise<nav_msgs::Odometry>("balloon_odom_out", 1);

  //}

  /* profiler //{ */

  m_profiler = new mrs_lib::Profiler(nh_, "BalloonPlanner", false);

  //}

  /* dynamic reconfigure //{ */

  /* reconfigure_server_.reset(new ReconfigureServer(config_mutex_, nh_)); */
  /* ReconfigureServer::CallbackType f = boost::bind(&CvDepth::callbackReconfigure, this, _1, _2); */
  /* reconfigure_server_->setCallback(f); */

  //}

  m_is_initialized = true;

  ROS_INFO("[BalloonPlanner]: initialized");
}

//}

/* callbackBalloonCloud() //{ */

void BalloonPlanner::callbackBalloonCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& msg) {

  if (!m_is_initialized)
    return;

  ROS_DEBUG_THROTTLE(1.0, "[BalloonPlanner]: Obtained balloon cloud");

  mrs_lib::Routine profiler_routine = m_profiler->createRoutine("callbackBalloonCloud");

  ROS_INFO("[BalloonPlanner]: balloon cloud size: %lu", msg->size());

}

//}

/* //{ callbackReconfigure() */

/* void BalloonPlanner::callbackReconfigure([[maybe_unused]] balloon_planner::bpConfig& config, [[maybe_unused]] uint32_t level) { */

/*   if (!m_is_initialized) */
/*     return; */

/*   ROS_INFO( */
/*       "Reconfigure Request: " */
/*       "median_blur: %d, scale: %f, range_min: %f, range_max: %f", */
/*       config.median_blur, config.scale, config.range_min, config.range_max); */

/*   _median_blur = config.median_blur; */
/*   _scale       = config.scale; */
/*   _range_min   = config.range_min; */
/*   _range_max   = config.range_max; */
/* } */
//}

}  // namespace balloon_planner

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(balloon_planner::BalloonPlanner, nodelet::Nodelet)
