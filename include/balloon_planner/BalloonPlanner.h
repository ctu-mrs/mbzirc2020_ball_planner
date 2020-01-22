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
#include <mrs_msgs/MpcTrackerDiagnostics.h>

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
#include <balloon_filter/BallLocation.h>
#include <balloon_filter/BallPrediction.h>
#include <balloon_planner/PlanningParamsConfig.h>
#include <balloon_filter/ResetEstimates.h>
#include <mrs_msgs/TrackerTrajectory.h>

//}

#define MSG_THROTTLE 0.5

namespace balloon_planner
{
  // shortcut type to the dynamic reconfigure manager template instance
  using drcfg_t = balloon_planner::PlanningParamsConfig;
  using drmgr_t = mrs_lib::DynamicReconfigureMgr<drcfg_t>;

  using vec2_t = Eigen::Vector2d;
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
      following_prediction,
      going_to_lurk,
      lurking,
    };
  }
  using state_t = state_enum::state_t;

  struct plane_t
  {
    vec3_t point;
    vec3_t normal;
  };

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

      double m_approach_speed;
      double m_chase_speed;
      double m_max_unseen_time;

      double m_trajectory_sampling_dt;
      double m_trajectory_horizon;

      vec4_t m_start_position;
      double m_target_offset;
      double m_catch_trigger_distance;

      int m_lurking_min_pts;
      ros::Duration m_lurking_min_dur;
      double m_lurking_observe_dist;
      double m_lurking_max_dist_from_trajectory;
      double m_lurking_max_reposition;

      double m_pid_kP;
      double m_pid_kI;
      double m_pid_kD;
      double m_pid_max_I;
      ros::Duration m_pid_reset_duration;

      //}

      /* ROS related variables (subscribers, timers etc.) //{ */
      std::unique_ptr<drmgr_t> m_drmgr_ptr;
      tf2_ros::Buffer m_tf_buffer;
      std::unique_ptr<tf2_ros::TransformListener> m_tf_listener_ptr;
      mrs_lib::SubscribeHandlerPtr<balloon_filter::BallLocation> m_sh_ball_detection;
      mrs_lib::SubscribeHandlerPtr<balloon_filter::BallPrediction> m_sh_ball_prediction;
      mrs_lib::SubscribeHandlerPtr<nav_msgs::Odometry> m_sh_cmd_odom;
      mrs_lib::SubscribeHandlerPtr<nav_msgs::Odometry> m_sh_main_odom;
      mrs_lib::SubscribeHandlerPtr<mrs_msgs::MpcTrackerDiagnostics> m_sh_tracker_diags;

      ros::Publisher m_pub_cmd_traj;
      ros::Publisher m_pub_dbg_traj;
      ros::Publisher m_pub_dbg_ball_positions;
      ros::Publisher m_pub_dbg_lurking_points;

      ros::ServiceClient m_srv_reset_filter;

      ros::Timer m_main_loop_timer;
      //}

      size_t m_max_pts;

    private:
      state_t m_state;
      double m_path_offset;
      ros::Time m_following_start;
      std::vector<vec3_t> m_ball_positions;
      vec4_t m_orig_lurk_pose;

      // --------------------------------------------------------------
      // |                helper implementation methods               |
      // --------------------------------------------------------------

      void reset_filter();

      std::optional<vec3_t> calc_ball_image_pos(const balloon_filter::BallLocation& ball_filtered);
      double pid(const double error, const ros::Time& stamp);

      vec3_t calc_horizontal_offset_vector(const vec3_t& dir_vec, const double tolerance = 1e-9);

      std::optional<Eigen::Affine3d> get_transform(const std::string& from_frame_id, const std::string& to_frame_id, ros::Time stamp);
      std::optional<geometry_msgs::TransformStamped> get_transform_raw(const std::string& from_frame_id, const std::string& to_frame_id, ros::Time stamp);
      std::optional<Eigen::Affine3d> get_transform_to_world(const std::string& frame_id, ros::Time stamp);
      std::optional<vec4_t> get_current_position();
      std::optional<vec4_t> get_current_cmd_position();

      vec4_t choose_lurking_pose(const std::vector<vec3_t>& ball_positions);
      plane_t get_yz_plane(const vec3_t& pos, const double yaw);
      vec3_t path_plane_intersection(const path_t& path, const plane_t& plane);

      path_t offset_path(const path_t& path, const vec3_t& off_vec);
      vec3_t find_approach_pt(const vec3_t& from_pt, const ros::Time& from_time, const path_t& to_path, const double speed);
      ros::Duration trajectory_duration(const int n_traj_pts, const double dt);
      traj_t sample_trajectory_between_pts(const vec3_t& from_pt, const vec3_t& to_pt, const double speed, const double dt, const double yaw = 0.0, const std_msgs::Header& header = {});
      traj_t sample_trajectory_from_path(const path_t& path, const double dt, const double speed, const size_t n_pts, const std_msgs::Header& header = {});
      traj_t sample_trajectory_from_path(const ros::Time& start_stamp, const path_t& path, const double dt, const size_t n_pts);
      traj_t join_trajectories(const traj_t& traj1, const traj_t& traj2);
      traj_t orient_trajectory_yaw(const traj_t& traj, const vec3_t& to_point);
      traj_t orient_trajectory_yaw_observe(const traj_t& traj, const path_t& to_path);
      traj_t orient_trajectory_yaw_speed(const traj_t& traj, const path_t& to_path);
      static vec3_t limit_cmd_vec_speed(const vec3_t& cmd_vector, const vec3_t& max_speed, const double dt, size_t max_pts);
      traj_t point_to_traj(const vec3_t& point, const size_t n_pts);
      sensor_msgs::PointCloud2 to_output_message(const std::vector<vec3_t>& points, const std_msgs::Header& header);

      path_t traj_to_path(const traj_t& traj, const double traj_dt);
      traj_t path_to_traj(const path_t& path);

      void load_dynparams(drcfg_t cfg);

  };
  
  //}

}  // namespace balloon_planner

#endif
