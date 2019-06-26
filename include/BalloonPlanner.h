#ifndef BALLOONPLANNER_H
#define BALLOONPLANNER_H

/* includes //{ */

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/PointCloud.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Geometry msgs
#include <geometry_msgs/TransformStamped.h>
/* #include <geometry_msgs/PoseWithCovarianceStamped.h> */
#include <geometry_msgs/PoseStamped.h>

// Nav msgs
#include <nav_msgs/Odometry.h>

// MRS stuff
#include <mrs_lib/Profiler.h>
#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/SubscribeHandler.h>
#include <mrs_lib/DynamicReconfigureMgr.h>

// std
#include <string>
#include <mutex>

#include <balloon_planner/PlanningParamsConfig.h>
#include <balloon_planner/ResetChosen.h>

//}

namespace balloon_planner
{
  // shortcut type to the dynamic reconfigure manager template instance
  using drcfg_t = balloon_planner::PlanningParamsConfig;
  using drmgr_t = mrs_lib::DynamicReconfigureMgr<drcfg_t>;

  /* //{ class BalloonPlanner */

  class BalloonPlanner : public nodelet::Nodelet
  {
    public:
      BalloonPlanner() : m_node_name("BalloonPlanner") {};
      virtual void onInit();

      bool m_is_initialized;

    private:
      const std::string m_node_name;
      void main_loop([[maybe_unused]] const ros::TimerEvent& evt);

    private:
      std::unique_ptr<mrs_lib::Profiler> m_profiler_ptr;

    private:

      // --------------------------------------------------------------
      // |                ROS-related member variables                |
      // --------------------------------------------------------------

      /* Parameters, loaded from ROS //{ */
      std::string m_world_frame;
      std::string m_uav_frame_id;
      double m_min_balloon_height;
      double m_filter_coeff;
      double m_gating_distance;
      double m_max_time_since_update;
      double m_min_updates_to_confirm;
      //}

      /* ROS related variables (subscribers, timers etc.) //{ */
      std::unique_ptr<drmgr_t> m_drmgr_ptr;
      tf2_ros::Buffer m_tf_buffer;
      std::unique_ptr<tf2_ros::TransformListener> m_tf_listener_ptr;
      mrs_lib::SubscribeHandlerPtr<sensor_msgs::PointCloud> m_sh_balloons;

      ros::Publisher m_pub_chosen_balloon;

      ros::ServiceServer m_reset_chosen_server;

      ros::Timer m_main_loop_timer;
      //}

      bool m_current_estimate_exists;
      Eigen::Vector3d m_current_estimate;
      ros::Time m_current_estimate_last_update;
      int m_current_estimate_n_updates;

    private:

      // --------------------------------------------------------------
      // |                helper implementation methods               |
      // --------------------------------------------------------------

      /* get_transform_to_world() method //{ */
      bool get_transform_to_world(const std::string& frame_name, ros::Time stamp, Eigen::Affine3d& tf_out)
      {
        try
        {
          const ros::Duration timeout(1.0 / 100.0);
          geometry_msgs::TransformStamped transform = m_tf_buffer.lookupTransform(m_world_frame, frame_name, stamp, timeout);
          tf_out = tf2::transformToEigen(transform.transform);
        } catch (tf2::TransformException& ex)
        {
          ROS_WARN("Error during transform from \"%s\" frame to \"%s\" frame.\n\tMSG: %s", frame_name.c_str(), m_world_frame.c_str(), ex.what());
          return false;
        }
        return true;
      }
      //}

      bool point_valid(const Eigen::Vector3d& pt, float dist_quality);

      void reset_current_estimate();
      geometry_msgs::PoseStamped to_output_message(const Eigen::Vector3d& position_estimate, const std_msgs::Header& header);
      Eigen::Vector3d get_cur_mav_pos();
      bool find_closest_to(const std::vector<Eigen::Vector3d>& balloons_positions, const Eigen::Vector3d& to_position, Eigen::Vector3d& closest_out, bool use_gating = false);
      bool find_closest(const std::vector<Eigen::Vector3d>& balloons_positions, Eigen::Vector3d& closest_out);

      std::vector<Eigen::Vector3d> message_to_positions(const sensor_msgs::PointCloud& balloon_msg);

      bool reset_chosen_callback(balloon_planner::ResetChosen::Request& req, balloon_planner::ResetChosen::Response& resp);
      void load_dynparams(drcfg_t cfg);

  };
  
  //}

}  // namespace balloon_planner

#endif
