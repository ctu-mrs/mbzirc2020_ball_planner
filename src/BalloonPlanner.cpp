#include "BalloonPlanner.h"

namespace balloon_planner
{

  /* main_loop() method //{ */
  void BalloonPlanner::main_loop([[maybe_unused]] const ros::TimerEvent& evt)
  {
    if (m_sh_balloons->new_data())
    {
      sensor_msgs::PointCloud balloons = m_sh_balloons->get_data();
  
      ROS_INFO_THROTTLE(1.0, "[%s]: Processing %lu new detections", m_node_name.c_str(), balloons.points.size());

      if (balloons.points.size() > 0)
      {
        auto balloons_positions = message_to_positions(balloons);
        bool meas_valid = false;
        Eigen::Vector3d closest_balloon;
        if (m_current_estimate_exists)
        {
          meas_valid = find_closest_to(balloons_positions, m_current_estimate, closest_balloon, true);
          if (meas_valid)
          {
            m_current_estimate = m_filter_coeff*m_current_estimate + (1.0 - m_filter_coeff)*closest_balloon;
            m_current_estimate_last_update = balloons.header.stamp;
            m_current_estimate_n_updates++;
          }
        } else
        {
          meas_valid = find_closest(balloons_positions, closest_balloon);
          if (meas_valid)
          {
            m_current_estimate = closest_balloon;
            m_current_estimate_exists = true;
            m_current_estimate_last_update = balloons.header.stamp;
            m_current_estimate_n_updates = 1;
          }
        }
      }
      ros::Duration del = ros::Time::now() - balloons.header.stamp;
      ROS_INFO_STREAM_THROTTLE(1.0, "delay (from image acquisition): " << del.toSec() * 1000.0 << "ms");
      ROS_INFO_THROTTLE(1.0, "[%s]: New data processed", m_node_name.c_str());
    }

    if ((ros::Time::now() - m_current_estimate_last_update).toSec() >= m_max_time_since_update)
    {
      reset_current_estimate();
    }

    if (m_current_estimate_exists && m_current_estimate_n_updates > m_min_updates_to_confirm)
    {
      std_msgs::Header header;
      header.frame_id = m_world_frame;
      header.stamp = m_current_estimate_last_update;
      m_pub_chosen_balloon.publish(to_output_message(m_current_estimate, header));
      ROS_INFO_THROTTLE(1.0, "[%s]: Current chosen balloon position: [%.2f, %.2f, %.2f]", m_node_name.c_str(), m_current_estimate.x(), m_current_estimate.y(), m_current_estimate.z());
    }
  }
  //}

  geometry_msgs::PoseStamped BalloonPlanner::to_output_message(const Eigen::Vector3d& position_estimate, const std_msgs::Header& header)
  {
    geometry_msgs::PoseStamped ret;

    ret.header = header;
    ret.pose.position.x = position_estimate.x();
    ret.pose.position.y = position_estimate.y();
    ret.pose.position.z = position_estimate.z();

    return ret;
  }

  Eigen::Vector3d BalloonPlanner::get_cur_mav_pos()
  {
    Eigen::Affine3d m2w_tf;
    bool tf_ok = get_transform_to_world(m_uav_frame_id, ros::Time::now(), m2w_tf);
    if (!tf_ok)
      return Eigen::Vector3d(0, 0, 0);;
    return m2w_tf.translation();
  }

  bool BalloonPlanner::find_closest_to(const std::vector<Eigen::Vector3d>& balloons_positions, const Eigen::Vector3d& to_position, Eigen::Vector3d& closest_out, bool use_gating)
  {
    double min_dist = std::numeric_limits<double>::infinity();
    Eigen::Vector3d closest_pt(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN());
    for (const auto& pt : balloons_positions)
    {
      const double cur_dist = (to_position - pt).norm();
      if (cur_dist < min_dist)
      {
        min_dist = cur_dist;
        closest_pt = pt;
      }
    }
    if (use_gating)
    {
      if (min_dist < m_gating_distance)
      {
        closest_out = closest_pt;
        return true;
      } else
      {
        return false;
      }
    } else
    {
      closest_out = closest_pt;
      return true;
    }
  }

  bool BalloonPlanner::find_closest(const std::vector<Eigen::Vector3d>& balloons_positions, Eigen::Vector3d& closest_out)
  {
    Eigen::Vector3d cur_pos = get_cur_mav_pos();
    return find_closest_to(balloons_positions, cur_pos, closest_out, false);
  }

  /* message_to_positions() method //{ */
  std::vector<Eigen::Vector3d> BalloonPlanner::message_to_positions(const sensor_msgs::PointCloud& balloon_msg)
  {
    std::vector<Eigen::Vector3d> ret;
  
    // Construct a new world to sensor transform
    Eigen::Affine3d s2w_tf;
    bool tf_ok = get_transform_to_world(balloon_msg.header.frame_id, balloon_msg.header.stamp, s2w_tf);
    if (!tf_ok)
      return ret;
  
    ret.reserve(balloon_msg.points.size());
    for (size_t it = 0; it < balloon_msg.points.size(); it++)
    {
      const auto rpt = balloon_msg.points[it];
      const auto dist_qual = balloon_msg.channels.at(0).values.at(it);
      if (point_valid(rpt, dist_qual))
      {
        Eigen::Vector3d pt(rpt.x, rpt.y, rpt.z);
        pt = s2w_tf*pt;
        ret.push_back(pt);
      }
    }
  
    return ret;
  }
  //}

  /* point_valid() method //{ */
  bool BalloonPlanner::point_valid(const geometry_msgs::Point32& pt, float dist_quality)
  {
    const bool height_valid = pt.z > m_min_balloon_height;
    const bool dist_valid = dist_quality == 3.0f;
  
    return height_valid && dist_valid;
  }
  //}

  void BalloonPlanner::reset_current_estimate()
  {
    m_current_estimate_exists = false;
    m_current_estimate_last_update = ros::Time::now();
    m_current_estimate_n_updates = 0;
  }

/* onInit() //{ */

void BalloonPlanner::onInit()
{

  ROS_INFO("[%s]: Initializing", m_node_name.c_str());
  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
  ros::Time::waitForValid();

  /* load parameters //{ */

  mrs_lib::ParamLoader pl(nh, m_node_name);

  double planning_period = pl.load_param2<double>("planning_period");
  pl.load_param("world_frame", m_world_frame);
  pl.load_param("uav_frame_id", m_uav_frame_id);
  pl.load_param("min_balloon_height", m_min_balloon_height);
  pl.load_param("filter_coeff", m_filter_coeff);
  pl.load_param("gating_distance", m_gating_distance);
  pl.load_param("max_time_since_update", m_max_time_since_update);
  pl.load_param("min_updates_to_confirm", m_min_updates_to_confirm);

  if (!pl.loaded_successfully())
  {
    ROS_ERROR("Some compulsory parameters were not loaded successfully, ending the node");
    ros::shutdown();
  }

  // LOAD DYNAMIC PARAMETERS
  m_drmgr_ptr = std::make_unique<drmgr_t>(nh, m_node_name);
  if (!m_drmgr_ptr->loaded_successfully())
  {
    ROS_ERROR("Some dynamic parameter default values were not loaded successfully, ending the node");
    ros::shutdown();
  }

  //}

  /* subscribers //{ */

  m_tf_listener_ptr = std::make_unique<tf2_ros::TransformListener>(m_tf_buffer, m_node_name);
  mrs_lib::SubscribeMgr smgr(nh);
  m_sh_balloons = smgr.create_handler_threadsafe<sensor_msgs::PointCloud>("balloon_cloud_in", 10, ros::TransportHints().tcpNoDelay(), ros::Duration(5.0));

  if (!smgr.loaded_successfully())
  {
    ROS_ERROR("Unable to subscribe to some topics, ending the node");
    ros::shutdown();
  }

  //}

  /* publishers //{ */

  m_pub_chosen_balloon = nh.advertise<geometry_msgs::PointStamped>("balloon_chosen_out", 1);

  //}

  /* profiler //{ */

  m_profiler_ptr = std::make_unique<mrs_lib::Profiler>(nh, m_node_name, false);

  //}

  reset_current_estimate();
  m_is_initialized = true;

  /* timers  //{ */

  m_main_loop_timer = nh.createTimer(ros::Duration(planning_period), &BalloonPlanner::main_loop, this);

  //}

  ROS_INFO("[%s]: initialized", m_node_name.c_str());
}

//}

}  // namespace balloon_planner

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(balloon_planner::BalloonPlanner, nodelet::Nodelet)
