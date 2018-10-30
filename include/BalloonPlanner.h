#ifndef BALLOONPLANNER_H
#define BALLOONPLANNER_H

/* includes //{ */

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

// Sensor msgs
#include <sensor_msgs/PointCloud.h>

// Geometry msgs
/* #include <geometry_msgs/Point32.h> */
/* #include <geometry_msgs/TransformStamped.h> */

// Nav msgs
#include <nav_msgs/Odometry.h>

// MRS stuff
#include <mrs_lib/Profiler.h>
#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/SubscribeHandler.h>

// Dynamic reconfigure
/* #include <dynamic_reconfigure/server.h> */
/* #include <balloon_planner/bpConfig.h> */

// std
#include <string>
#include <mutex>

//}

namespace balloon_planner
{

  /* //{ class BalloonPlanner */

  class BalloonPlanner : public nodelet::Nodelet
  {

    public:
      BalloonPlanner() : m_node_name("BalloonPlanner") {};
      virtual void onInit();

    public:
      bool m_is_initialized = false;

    private:
      const std::string m_node_name;
    private:
      mrs_lib::SubscribeHandlerPtr<sensor_msgs::PointCloud> m_sh_balloons;

      ros::Publisher m_pub_odom_balloon;

    private:
      std::unique_ptr<mrs_lib::Profiler> m_profiler_ptr;

      /* parameters //{ */

    private:

      //}

      /* /1* message callbacks //{ *1/ */

      /* void callbackBalloonCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& msg); */

      /* //} */

      /* /1* dynamic reconfigure //{ *1/ */

      /* /1* boost::recursive_mutex                      config_mutex_; *1/ */
      /* /1* typedef balloon_planner::bpConfig             Config; *1/ */
      /* /1* typedef dynamic_reconfigure::Server<Config> ReconfigureServer; *1/ */
      /* /1* boost::shared_ptr<ReconfigureServer>        reconfigure_server_; *1/ */
      /* /1* balloon_planner::bpConfig                     last_drs_config; *1/ */

      /* /1* void callbackReconfigure([[maybe_unused]] balloon_planner::bpConfig& config, [[maybe_unused]] uint32_t level); *1/ */

      /* //} */

  };


  //}

}  // namespace balloon_planner

#endif
