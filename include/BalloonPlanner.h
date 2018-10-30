#ifndef BALLOONPLANNER_H
#define BALLOONPLANNER_H

/* includes //{ */

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/PointCloud.h>

// Geometry msgs
#include <geometry_msgs/TransformStamped.h>

// Nav msgs
#include <nav_msgs/Odometry.h>

// MRS stuff
#include <mrs_lib/Profiler.h>
#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/SubscribeHandler.h>

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

      bool m_is_initialized;

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

  };
  
  //}

}  // namespace balloon_planner

#endif
