#include <balloon_planner/BalloonPlanner.h>

namespace balloon_planner
{

  /* main_loop() method //{ */
  void BalloonPlanner::main_loop([[maybe_unused]] const ros::TimerEvent& evt)
  {
    load_dynparams(m_drmgr_ptr->config);

    if (m_sh_balloons->new_data())
    {
      const auto balloons = m_sh_balloons->get_data();
  
      ROS_INFO_THROTTLE(1.0, "[%s]: Processing %lu new detections", m_node_name.c_str(), balloons.poses.size());

      if (balloons.poses.size() > 0)
      {
        auto measurements = message_to_positions(balloons);
        if (m_current_estimate_exists)
          update_current_estimate(measurements, balloons.header.stamp);
        else
          init_current_estimate(measurements, balloons.header.stamp);
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
      m_pub_chosen_balloon.publish(to_output_message(m_current_estimate.x, header));
      ROS_INFO_THROTTLE(1.0, "[%s]: Current chosen balloon position: [%.2f, %.2f, %.2f]", m_node_name.c_str(), m_current_estimate.x.x(), m_current_estimate.x.y(), m_current_estimate.x.z());
    }
  }
  //}

  /* update_current_estimate() method //{ */
  void BalloonPlanner::update_current_estimate(const std::vector<pos_cov_t>& measurements, const ros::Time& stamp)
  {
    pos_cov_t closest_meas;
    bool meas_valid = find_closest_to(measurements, m_current_estimate.x, closest_meas, true);
    if (meas_valid)
    {
      const double dt = (stamp - m_current_estimate_last_update).toSec();
      m_current_estimate.R = dt*m_process_noise_std*Lkf::R_t::Identity();
      m_current_estimate.prediction_step();
      m_current_estimate.z = closest_meas.pos;
      m_current_estimate.correction_step();
      /* m_current_estimate = m_filter_coeff*m_current_estimate + (1.0 - m_filter_coeff)*closest_balloon; */
      m_current_estimate_last_update = stamp;
      m_current_estimate_n_updates++;
    }
  }
  //}

  /* init_current_estimate() method //{ */
  void BalloonPlanner::init_current_estimate(const std::vector<pos_cov_t>& measurements, const ros::Time& stamp)
  {
    pos_cov_t closest_meas;
    bool meas_valid = find_closest(measurements, closest_meas);
    if (meas_valid)
    {
      m_current_estimate.x = closest_meas.pos;
      m_current_estimate.P = closest_meas.cov;
      m_current_estimate_exists = true;
      m_current_estimate_last_update = stamp;
      m_current_estimate_n_updates = 1;
    }
  }
  //}

  /* to_output_message() method //{ */
  geometry_msgs::PoseStamped BalloonPlanner::to_output_message(const pos_t& estimate, const std_msgs::Header& header)
  {
    geometry_msgs::PoseStamped ret;
  
    ret.header = header;
    ret.pose.position.x = estimate.x();
    ret.pose.position.y = estimate.y();
    ret.pose.position.z = estimate.z();
  
    return ret;
  }
  //}

  /* get_cur_mav_pos() method //{ */
  pos_t BalloonPlanner::get_cur_mav_pos()
  {
    Eigen::Affine3d m2w_tf;
    bool tf_ok = get_transform_to_world(m_uav_frame_id, ros::Time::now(), m2w_tf);
    if (!tf_ok)
      return pos_t(0, 0, 0);;
    return m2w_tf.translation();
  }
  //}

  /* find_closest_to() method //{ */
  bool BalloonPlanner::find_closest_to(const std::vector<pos_cov_t>& measurements, const pos_t& to_position, pos_cov_t& closest_out, bool use_gating)
  {
    double min_dist = std::numeric_limits<double>::infinity();
    pos_cov_t closest_pt{
       std::numeric_limits<double>::quiet_NaN()*pos_t::Ones(),
       std::numeric_limits<double>::quiet_NaN()*cov_t::Ones()
    };
    for (const auto& pt : measurements)
    {
      const double cur_dist = (to_position - pt.pos).norm();
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
  //}

  /* find_closest() method //{ */
  bool BalloonPlanner::find_closest(const std::vector<pos_cov_t>& measuremets, pos_cov_t& closest_out)
  {
    pos_t cur_pos = get_cur_mav_pos();
    return find_closest_to(measuremets, cur_pos, closest_out, false);
  }
  //}

  /* message_to_positions() method //{ */
  std::vector<pos_cov_t> BalloonPlanner::message_to_positions(const detections_t& balloon_msg)
  {
    std::vector<pos_cov_t> ret;
  
    // Construct a new world to sensor transform
    Eigen::Affine3d s2w_tf;
    bool tf_ok = get_transform_to_world(balloon_msg.header.frame_id, balloon_msg.header.stamp, s2w_tf);
    if (!tf_ok)
      return ret;
    const Eigen::Matrix3d s2w_rot = s2w_tf.rotation();
  
    ret.reserve(balloon_msg.poses.size());
    for (size_t it = 0; it < balloon_msg.poses.size(); it++)
    {
      const auto msg_pos = balloon_msg.poses[it].pose;
      const auto msg_cov = balloon_msg.poses[it].covariance;
      const pos_t pos = s2w_tf*pos_t(msg_pos.position.x, msg_pos.position.y, msg_pos.position.z);
      if (point_valid(pos))
      {
        const cov_t cov = rotate_covariance(msg2cov(msg_cov), s2w_rot);
        const pos_cov_t pos_cov{pos, cov};
        ret.push_back(pos_cov);
      }
    }
  
    return ret;
  }
  //}

  /* msg2cov() method //{ */
  cov_t BalloonPlanner::msg2cov(const ros_cov_t& msg_cov)
  {
    cov_t cov;
    for (int r = 0; r < 3; r++)
    {
      for (int c = 0; c < 3; c++)
      {
        cov(r, c) = msg_cov[r * 6 + c];
      }
    }
    return cov;
  }
  //}

  /* rotate_covariance() method //{ */
  cov_t BalloonPlanner::rotate_covariance(const cov_t& covariance, const cov_t& rotation)
  {
    return rotation * covariance * rotation.transpose();  // rotate the covariance to point in direction of est. position
  }
  //}


  /* point_valid() method //{ */
  bool BalloonPlanner::point_valid(const pos_t& pt)
  {
    const bool height_valid = pt.z() > m_min_balloon_height;
  
    return height_valid;
  }
  //}

  /* reset_current_estimate() method //{ */
  void BalloonPlanner::reset_current_estimate()
  {
    m_current_estimate_exists = false;
    m_current_estimate_last_update = ros::Time::now();
    m_current_estimate_n_updates = 0;
    ROS_INFO("[%s]: Current chosen balloon reset.", m_node_name.c_str());
  }
  //}

  /* load_dynparams() method //{ */
  void BalloonPlanner::load_dynparams(drcfg_t cfg)
  {
    m_min_balloon_height = cfg.min_balloon_height;
    m_filter_coeff = cfg.filter_coeff;
    m_gating_distance = cfg.gating_distance;
    m_max_time_since_update = cfg.max_time_since_update;
    m_min_updates_to_confirm = cfg.min_updates_to_confirm;
  }
  //}

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
  constexpr bool time_consistent = true;
  m_sh_balloons = smgr.create_handler_threadsafe<detections_t, time_consistent>("balloon_detections", 10, ros::TransportHints().tcpNoDelay(), ros::Duration(5.0));

  if (!smgr.loaded_successfully())
  {
    ROS_ERROR("Unable to subscribe to some topics, ending the node");
    ros::shutdown();
  }

  m_reset_chosen_server = nh.advertiseService("reset_chosen", &BalloonPlanner::reset_chosen_callback, this);
  //}

  /* publishers //{ */

  m_pub_chosen_balloon = nh.advertise<geometry_msgs::PointStamped>("balloon_chosen_out", 1);

  //}

  /* profiler //{ */

  m_profiler_ptr = std::make_unique<mrs_lib::Profiler>(nh, m_node_name, false);

  //}

  {
    Lkf::A_t A = Lkf::A_t::Identity();
    Lkf::B_t B;
    Lkf::H_t H = Lkf::H_t::Identity();
    Lkf::P_t P = std::numeric_limits<double>::quiet_NaN()*Lkf::P_t::Identity();
    Lkf::Q_t Q = m_process_noise_std*m_process_noise_std*Lkf::Q_t::Identity();
    Lkf::R_t R = std::numeric_limits<double>::quiet_NaN()*Lkf::R_t::Identity();
    m_current_estimate = Lkf(A, B, H, P, Q, R);
  }
  reset_current_estimate();
  m_is_initialized = true;

  /* timers  //{ */

  m_main_loop_timer = nh.createTimer(ros::Duration(planning_period), &BalloonPlanner::main_loop, this);

  //}

  ROS_INFO("[%s]: initialized", m_node_name.c_str());
}

//}

  /* BalloonPlanner::reset_chosen_callback() method //{ */
  
  bool BalloonPlanner::reset_chosen_callback(balloon_planner::ResetChosen::Request& req, balloon_planner::ResetChosen::Response& resp)
  {
    reset_current_estimate();
    resp.message = "Current chosen balloon was reset.";
    resp.success = true;
    return true;
  }
  
  //}

}  // namespace balloon_planner

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(balloon_planner::BalloonPlanner, nodelet::Nodelet)
