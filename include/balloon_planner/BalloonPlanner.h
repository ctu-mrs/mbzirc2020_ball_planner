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
#include <mrs_msgs/String.h>
#include <mrs_msgs/SetInt.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
#include <std_srvs/Trigger.h>

// OpenCV includes
#include <opencv2/imgproc/imgproc.hpp>

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
      /* lost_glancing, */
      waiting_for_detection,
      observing,
      /* yawing_detection, */
      /* following_detection, */
      /* following_prediction, */
      going_to_lurk,
      lurking,
    };
  }
  using state_t = state_enum::state_t;
  /* to_string //{ */
  
  std::string to_string(const state_t state)
  {
    switch (state)
    {
      /* case state_t::lost_glancing: return "lost_glancing"; */
      case state_t::waiting_for_detection: return "waiting_for_detection";
      case state_t::observing: return "observing";
      /* case state_t::yawing_detection: return "yawing_detection"; */
      /* case state_t::following_detection: return "following_detection"; */
      /* case state_t::following_prediction: return "following_prediction"; */
      case state_t::going_to_lurk: return "going_to_lurk";
      case state_t::lurking: return "lurking";
      default: return "unknown_state";
    }
  }
  
  //}

  namespace strat_enum
  {
    enum strat_t
    {
      lurk_arc_endpose,
      lurk_most_probable,
      lurk_lowest,
      unknown
    };
  }
  using strat_t = strat_enum::strat_t;
  /* to_string //{ */
  
  std::string to_string(const strat_t strat)
  {
    switch (strat)
    {
      case strat_t::lurk_arc_endpose: return "lurk_arc_endpose";
      case strat_t::lurk_most_probable: return "lurk_most_probable";
      case strat_t::lurk_lowest: return "lurk_lowes";
      default: return "unknown strategy";
    }
  }
  
  //}

  struct plane_t
  {
    vec3_t point;
    vec3_t normal;
  };

  struct pose_stamped_t
  {
    vec4_t pose;
    ros::Time stamp;
  };

  /* //{ class BalloonPlanner */

  class BalloonPlanner : public nodelet::Nodelet
  {
    public:
      BalloonPlanner() : m_node_name("BalloonPlanner"), m_initialized(false), m_activated(false) {};
      virtual void onInit();

      bool m_is_initialized;

    private:
      const std::string m_node_name;
      void main_loop([[maybe_unused]] const ros::TimerEvent& evt);
      void delayed_init([[maybe_unused]] const ros::TimerEvent& evt);

    private:
      std::unique_ptr<mrs_lib::Profiler> m_profiler_ptr;

    private:

      // --------------------------------------------------------------
      // |                ROS-related member variables                |
      // --------------------------------------------------------------

      /* Parameters, loaded from ROS //{ */
      std::string m_world_frame_id;
      std::string m_arena_frame_id;

      bool m_land_at_end;
      vec2_t m_land_zone;
      double m_landing_height;

      double m_pogo_min_height;
      double m_pogo_max_height;
      double m_pogo_speed;

      ros::Duration m_max_unseen_dur;

      double m_trajectory_sampling_dt;
      double m_trajectory_horizon;

      vec4_t m_start_pose;

      double m_yawing_max_ball_dist;

      ros::Duration m_lurking_min_observing_dur;
      double m_lurking_reaction_dist;
      double m_lurking_max_reposition;
      int m_lurking_min_last_pts;
      double m_lurking_z_offset;
      ros::Duration m_lurking_min_last_dur;
      double m_lurking_passthrough_dist;

      /* std::map<double, std::string> m_constraint_ranges; */
      std::map<std::string, std::string> m_constraint_states;

      //}

      /* ROS related variables (subscribers, timers etc.) //{ */
      std::unique_ptr<drmgr_t> m_drmgr_ptr;
      tf2_ros::Buffer m_tf_buffer;
      std::unique_ptr<tf2_ros::TransformListener> m_tf_listener_ptr;
      mrs_lib::SubscribeHandlerPtr<geometry_msgs::PoseWithCovarianceStamped> m_sh_ball_detection;
      mrs_lib::SubscribeHandlerPtr<geometry_msgs::PoseStamped> m_sh_ball_passthrough;
      mrs_lib::SubscribeHandlerPtr<balloon_filter::BallPrediction> m_sh_ball_prediction;
      mrs_lib::SubscribeHandlerPtr<nav_msgs::Odometry> m_sh_cmd_odom;
      mrs_lib::SubscribeHandlerPtr<nav_msgs::Odometry> m_sh_main_odom;
      mrs_lib::SubscribeHandlerPtr<mrs_msgs::MpcTrackerDiagnostics> m_sh_tracker_diags;

      ros::Publisher m_pub_cmd_traj;
      ros::Publisher m_pub_dbg_traj;
      ros::Publisher m_pub_dbg_ball_positions;

      ros::Publisher m_pub_dbg_lurking_points;
      ros::Publisher m_pub_dbg_lurking_position;
      ros::Publisher m_pub_dbg_xy_plane;
      ros::Publisher m_pub_dbg_int_pt;

      ros::Publisher m_pub_dbg_linefit;
      ros::Publisher m_pub_dbg_linefit_pts;

      ros::ServiceServer m_srv_start;
      ros::ServiceServer m_srv_stop;

      ros::ServiceClient m_srv_reset_filter;
      ros::ServiceClient m_srv_set_constraints;
      ros::ServiceClient m_srv_land_there;

      ros::Timer m_main_loop_timer;
      ros::Timer m_delayed_init_timer;
      //}

      size_t m_max_pts;

    private:
      bool m_initialized;
      bool m_activated;

      state_t m_state;
      strat_t m_strat;

      // | -------------------------- pogo -------------------------- |
      double m_pogo_prev_height;
      double m_pogo_height_range;
      double m_pogo_direction;
      ros::Time m_pogo_prev_time;

      // | ----------------------- other shit ----------------------- |
      ros::Time m_observing_start;
      std::vector<pose_stamped_t> m_ball_positions;
      vec4_t m_orig_lurk_pose;       // the original lurking pose
      vec4_t m_cur_lurk_pose;        // where the drone currently wans to lurk (without the z offset)
      vec4_t m_cur_lurk_pose_offset; // offset in the z - direction by m_lurking_z_offset
      double m_last_seen_relative_yaw;
      bool m_sent_lurk_pos;
      ros::Time m_sent_lurk_pos_stamp;
      double m_prev_signed_ball_dist_set;
      double m_prev_signed_ball_dist;

      vec4_t m_landing_pose;

      // --------------------------------------------------------------
      // |                helper implementation methods               |
      // --------------------------------------------------------------

      /* std::string pick_constraints(const double ball_dist); */
      std::string pick_constraints(const state_t state);
      void set_constraints(const std::string& constraints_name);
      void reset_filter();
      bool land_there(const vec4_t& landing_pose);

      vec3_t calc_horizontal_offset_vector(const vec3_t& dir_vec, const double tolerance = 1e-9);

      std::optional<Eigen::Affine3d> get_transform(const std::string& from_frame_id, const std::string& to_frame_id, ros::Time stamp);
      std::optional<geometry_msgs::TransformStamped> get_transform_raw(const std::string& from_frame_id, const std::string& to_frame_id, ros::Time stamp);
      std::optional<Eigen::Affine3d> get_transform_to_world(const std::string& frame_id, ros::Time stamp);
      std::optional<path_t> process_path(const path_t& pred);
      std::optional<pose_stamped_t> process_detection(const geometry_msgs::PoseWithCovarianceStamped& det);
      std::optional<pose_stamped_t> process_detection(const geometry_msgs::PoseStamped& det);
      std::optional<vec4_t> process_odom(const nav_msgs::Odometry& det);
      std::optional<pose_stamped_t> get_ball_position();
      std::optional<pose_stamped_t> get_ball_passthrough();
      std::optional<vec4_t> get_uav_position();
      std::optional<vec4_t> get_uav_cmd_position();
      std::optional<balloon_filter::BallPrediction> get_ball_prediction();

      vec4_t choose_lurking_pose(const std::vector<pose_stamped_t>& ball_positions);
      plane_t get_yz_plane(const vec4_t& pose);
      double signed_point_plane_distance(const vec3_t& point, const plane_t& plane);
      std::optional<vec3_t> path_plane_intersection(const path_t& path, const plane_t& plane);

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
      sensor_msgs::PointCloud2 to_output_message(const std::vector<pose_stamped_t>& points, const std_msgs::Header& header);
      geometry_msgs::PoseStamped to_output_message(const vec4_t& position, const std_msgs::Header& header);
      sensor_msgs::PointCloud2 to_output_message(const std::vector<cv::Point3d>& points, const std_msgs::Header& header);
      visualization_msgs::Marker to_output_message(const cv::Vec6f& line, const std_msgs::Header& header);
      geometry_msgs::PointStamped to_msg(const vec3_t& point, const std_msgs::Header& header);
      visualization_msgs::MarkerArray plane_visualization(const plane_t& plane, const std_msgs::Header& header);

      path_t traj_to_path(const traj_t& traj, const double traj_dt);
      traj_t path_to_traj(const path_t& path);


      void load_dynparams(drcfg_t cfg);

      bool start_callback(mrs_msgs::SetInt::Request& req, mrs_msgs::SetInt::Response& resp);
      bool stop_callback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);

  };
  
  //}

}  // namespace balloon_planner

#endif
