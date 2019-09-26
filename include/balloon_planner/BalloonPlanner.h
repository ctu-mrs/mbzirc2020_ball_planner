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

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// Msgs
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// MRS stuff
#include <mrs_lib/Profiler.h>
#include <mrs_lib/ParamLoader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/DynamicReconfigureMgr.h>
#include <mrs_lib/geometry_utils.h>

// Boost
#include <boost/circular_buffer.hpp>

// std
#include <string>
#include <mutex>

// local includes
#include <balloon_planner/PlanningParamsConfig.h>
#include <balloon_planner/ResetChosen.h>
#include <balloon_filter/Plane.h>
#include <mrs_msgs/TrackerTrajectory.h>

//}

#define MSG_THROTTLE 0.5

namespace balloon_planner
{
  // shortcut type to the dynamic reconfigure manager template instance
  using drcfg_t = balloon_planner::PlanningParamsConfig;
  using drmgr_t = mrs_lib::DynamicReconfigureMgr<drcfg_t>;

  using vec3_t = Eigen::Vector3d;
  using quat_t = Eigen::Quaterniond;
  using path_t = nav_msgs::Path;
  using plane_t = balloon_filter::Plane;
  using traj_t = mrs_msgs::TrackerTrajectory;

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

      double m_path_offset;
      double m_approach_speed;

      double m_trajectory_sampling_dt;
      double m_trajectory_horizon;

      //}

      /* ROS related variables (subscribers, timers etc.) //{ */
      std::unique_ptr<drmgr_t> m_drmgr_ptr;
      tf2_ros::Buffer m_tf_buffer;
      std::unique_ptr<tf2_ros::TransformListener> m_tf_listener_ptr;
      mrs_lib::SubscribeHandlerPtr<nav_msgs::Path> m_sh_predicted_path;
      mrs_lib::SubscribeHandlerPtr<balloon_filter::Plane> m_sh_fitted_plane;

      ros::Publisher m_pub_cmd_traj;
      ros::Publisher m_pub_dbg_traj;
      ros::Publisher m_pub_dbg_approach_pt;

      ros::Timer m_main_loop_timer;
      //}

      size_t m_max_pts;

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

      std::optional<vec3_t> get_current_position();
      vec3_t calc_path_offset_vector(const plane_t& plane_params, const vec3_t& towards_pt, const double tolerance = 1e-9);
      path_t offset_path(const path_t& path, const vec3_t& vector, const double offset);
      vec3_t find_approach_pt(const vec3_t& from_pt, const ros::Time& from_time, const path_t& to_path, const double speed);
      std::tuple<traj_t, ros::Duration, path_t> sample_trajectory_between_pts(const vec3_t& from_pt, const vec3_t& to_pt, const double speed, const double dt);
      std::tuple<traj_t, ros::Duration, path_t> sample_trajectory_from_path(const ros::Time& start_stamp, const path_t& path, const double dt, const size_t n_pts);
      traj_t join_trajectories(const traj_t& traj1, const traj_t& traj2);
      traj_t orient_trajectory_yaw(const traj_t& traj, const path_t& to_path);

      path_t traj_to_path(const traj_t& traj, const double traj_dt);

      void load_dynparams(drcfg_t cfg);

  };
  
  //}

}  // namespace balloon_planner

#endif
