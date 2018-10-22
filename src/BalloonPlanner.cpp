#include "BalloonPlanner.h"

namespace balloon_planner
{

/* onInit() //{ */

void BalloonPlanner::onInit() {

  ROS_INFO("[BalloonPlanner]: Initializing");
  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();
  ros::Time::waitForValid();

  /* initialize variables //{ */

  m_closest_balloon_frame_ = "closest_balloon";

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

  ROS_INFO("aaaaa");
  if (!m_is_initialized)
    return;

  ROS_DEBUG_THROTTLE(1.0, "[BalloonPlanner]: Obtained balloon cloud");

  mrs_lib::Routine profiler_routine = m_profiler->createRoutine("callbackBalloonCloud");

  ROS_INFO_THROTTLE(1.0, "[BalloonPlanner]: balloon cloud size: %d", msg->size());

  pcl::PointCloud<pcl::PointXYZ>::iterator pt_closest_tmp = getClosestPt(msg);

  // Fill the transform
  geometry_msgs::TransformStamped transformStamped;

  /* transformStamped.header.stamp            = stamp; */
  transformStamped.header.stamp            = ros::Time::now();
  transformStamped.header.frame_id         = msg->header.frame_id;
  transformStamped.child_frame_id          = m_closest_balloon_frame_;
  transformStamped.transform.translation.x = pt_closest_tmp->x;
  transformStamped.transform.translation.y = pt_closest_tmp->y;
  transformStamped.transform.translation.z = pt_closest_tmp->z;
  transformStamped.transform.rotation.x    = 0.0;
  transformStamped.transform.rotation.y    = 0.0;
  transformStamped.transform.rotation.z    = 0.0;
  transformStamped.transform.rotation.w    = 1.0;

  // Publish the transform
  m_br_.sendTransform(transformStamped);
  ROS_DEBUG_THROTTLE(1.0, "[BalloonPlanner]: Transform sent.");

  // Publish the odometry message
  /* try { */
  /*   m_pub_odom_balloon_.publish(odom_balloon); */
  /* } */
  /* catch (...) { */
  /*   ROS_ERROR("Exception caught during publishing topic %s.", m_pub_odom_balloon_.getTopic().c_str()); */
  /* } */
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

/* getClosest() //{ */

pcl::PointCloud<pcl::PointXYZ>::iterator BalloonPlanner::getClosestPt(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_in) {
  double                                   dst_closest = DBL_MAX;
  pcl::PointCloud<pcl::PointXYZ>::iterator pt_closest;
  for (auto pt = pc_in->begin(); pt != pc_in->end(); pt++) {
    double dst_curr = std::pow(pt->x, 2) + std::pow(pt->y, 2) + std::pow(pt->z, 2);
    if (dst_curr < dst_closest) {
      pt_closest  = pt;
      dst_closest = dst_curr;
    }
  }
}

//}
}  // namespace balloon_planner

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(balloon_planner::BalloonPlanner, nodelet::Nodelet)
