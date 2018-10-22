#ifndef BALLOONPLANNER_H
#define BALLOONPLANNER_H

/* includes //{ */

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

//tf2
#include <tf2_ros/transform_broadcaster.h>


// Sensor msgs
#include <sensor_msgs/PointCloud.h>

// Geometry msgs
/* #include <geometry_msgs/Point32.h> */
#include <geometry_msgs/TransformStamped.h>

// Nav msgs
#include <nav_msgs/Odometry.h>

// MRS stuff
#include <mrs_lib/Profiler.h>
#include <mrs_lib/ParamLoader.h>

// Dynamic reconfigure
/* #include <dynamic_reconfigure/server.h> */
/* #include <balloon_planner/bpConfig.h> */

// std
#include <string>
#include <mutex>
#include <float.h>

//}

namespace balloon_planner
{

/* //{ class BalloonPlanner */

class BalloonPlanner : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool m_is_initialized = false;
  
  std::string m_closest_balloon_frame_;

  /* ROS //{ */
  
  private:
  
    // publishers
    ros::Subscriber m_sub_cloud_balloon_;
  
    //subscribers
    ros::Publisher m_pub_odom_balloon_;
  
    //tranforms
    tf2_ros::TransformBroadcaster m_br_;
  
  
  //}
  
private:
  mrs_lib::Profiler* m_profiler;

  /* parameters //{ */

private:

  //}

  /* message callbacks //{ */

  void callbackBalloonCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& msg);

  //}

  /* dynamic reconfigure //{ */

  /* boost::recursive_mutex                      config_mutex_; */
  /* typedef balloon_planner::bpConfig             Config; */
  /* typedef dynamic_reconfigure::Server<Config> ReconfigureServer; */
  /* boost::shared_ptr<ReconfigureServer>        reconfigure_server_; */
  /* balloon_planner::bpConfig                     last_drs_config; */

  /* void callbackReconfigure([[maybe_unused]] balloon_planner::bpConfig& config, [[maybe_unused]] uint32_t level); */

  //}

pcl::PointCloud<pcl::PointXYZ>::iterator getClosestPt(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_in);
};


//}

}  // namespace balloon_planner

#endif
