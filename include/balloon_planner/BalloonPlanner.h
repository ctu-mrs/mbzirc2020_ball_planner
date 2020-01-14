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
#include <sensor_msgs/Range.h>

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
#include <balloon_filter/BallPrediction.h>
#include <balloon_planner/PlanningParamsConfig.h>
#include <balloon_planner/ResetChosen.h>
#include <mrs_msgs/TrackerTrajectory.h>

//}

#define MSG_THROTTLE 0.5

namespace balloon_planner
{
  // shortcut type to the dynamic reconfigure manager template instance
  using drcfg_t = balloon_planner::PlanningParamsConfig;
  using drmgr_t = mrs_lib::DynamicReconfigureMgr<drcfg_t>;

  using vec3_t = Eigen::Vector3d;
  using vec4_t = Eigen::Vector4d;
  using quat_t = Eigen::Quaterniond;
  using path_t = nav_msgs::Path;
  using traj_t = mrs_msgs::TrackerTrajectory;

  namespace state_enum
  {
    enum state_t
    {
      waiting_for_detection,
      following_detection,
      chasing_prediction,
    };
  }
  using state_t = state_enum::state_t;

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
      std::string m_world_frame_id;
      std::string m_uav_frame_id;

      double m_approach_speed;
      double m_max_unseen_time;

      double m_trajectory_sampling_dt;
      double m_trajectory_horizon;

      vec4_t m_start_position;
      double m_observing_max_ball_angle;
      double m_observing_ball_distance;
      double m_observing_max_forward_speed;
      double m_target_offset;

      //}

      /* ROS related variables (subscribers, timers etc.) //{ */
      std::unique_ptr<drmgr_t> m_drmgr_ptr;
      tf2_ros::Buffer m_tf_buffer;
      std::unique_ptr<tf2_ros::TransformListener> m_tf_listener_ptr;
      mrs_lib::SubscribeHandlerPtr<geometry_msgs::PoseWithCovarianceStamped> m_sh_ball_detection;
      mrs_lib::SubscribeHandlerPtr<balloon_filter::BallPrediction> m_sh_ball_prediction;

      ros::Publisher m_pub_cmd_traj;
      ros::Publisher m_pub_dbg_traj;
      ros::Publisher m_pub_dbg_approach_traj;
      ros::Publisher m_pub_dbg_follow_traj;
      ros::Publisher m_pub_dbg_approach_pt;
      ros::Publisher m_pub_dbg_path_pose1;
      ros::Publisher m_pub_dbg_path_pose2;
      ros::Publisher m_pub_dbg_observe_cone;

      ros::Timer m_main_loop_timer;
      //}

      size_t m_max_pts;

    private:
      state_t m_state;
      double m_path_offset;
      ros::Time m_prev_plan_stamp;

      // --------------------------------------------------------------
      // |                helper implementation methods               |
      // --------------------------------------------------------------

      vec3_t calc_horizontal_offset_vector(const vec3_t& dir_vec, const double tolerance = 1e-9);

      std::optional<Eigen::Affine3d> get_transform(const std::string& from_frame_id, const std::string& to_frame_id, ros::Time stamp);
      std::optional<Eigen::Affine3d> get_transform_to_world(const std::string& frame_id, ros::Time stamp);
      std::optional<vec3_t> get_current_position();

      path_t offset_path(const path_t& path, const vec3_t& off_vec);
      vec3_t find_approach_pt(const vec3_t& from_pt, const ros::Time& from_time, const path_t& to_path, const double speed);
      std::tuple<traj_t, ros::Duration> sample_trajectory_between_pts(const vec3_t& from_pt, const vec3_t& to_pt, const double speed, const double dt, const double yaw = 0.0);
      std::tuple<traj_t, ros::Duration> sample_trajectory_from_path(const ros::Time& start_stamp, const path_t& path, const double dt, const size_t n_pts);
      traj_t join_trajectories(const traj_t& traj1, const traj_t& traj2);
      traj_t orient_trajectory_yaw(const traj_t& traj, const path_t& to_path);
      static vec3_t limit_cmd_vec_speed(const vec3_t& cmd_vector, const vec3_t& max_speed, const double dt, size_t max_pts);
      traj_t point_to_traj(const vec3_t& point, const size_t n_pts);

      path_t traj_to_path(const traj_t& traj, const double traj_dt);

      void load_dynparams(drcfg_t cfg);

  };
  
  //}

}  // namespace balloon_planner

#endif
