#include "BalloonPlanner.h"

namespace balloon_planner
{

void BalloonPlanner::main_loop(const ros::TimerEvent& evt)
{

  ros::Time start_t = ros::Time::now();

  if (m_sh_balloons->new_data())
  {
    sensor_msgs::PointCloud balloons = m_sh_balloons->get_data();

    ROS_INFO("[%s]: Processing %lu new detections", m_node_name.c_str(), balloons.points.size());

    if (balloons.header.frame_id == m_world_frame)
      ROS_WARN("[%s]: It seems that detections are not in the sensor frame (detection frame: %s, world frame: %s). The covariances may not be calculated correctly!",
               m_node_name.c_str(), balloons.header.frame_id.c_str(), m_world_frame.c_str());
    // Construct a new world to sensor transform
    Eigen::Affine3d s2w_tf;
    bool tf_ok = get_transform_to_world(balloons.header.frame_id, balloons.header.stamp, s2w_tf);

    if (!tf_ok)
      return;

    // keep track of how many LKFs are created and kicked out in this iteration
    int starting_lkfs = 0;
    int new_lkfs = 0;
    int kicked_out_lkfs = 0;
    int ending_lkfs = 0;
    // observer pointer, which will contain the most likely LKF (if any)
    Lkf const * most_certain_lkf = nullptr;
    /* Process the detections and update the LKFs, find the most certain LKF after the update, kick out uncertain LKFs //{ */
    // TODO: assignment problem?? (https://en.wikipedia.org/wiki/Hungarian_algorithm)
    {
      size_t n_meas = balloons.points.size();
      std::vector<int> meas_used(n_meas, 0);
      std::vector<pos_cov_t> pos_covs;
      pos_covs.reserve(n_meas);

      /* Calculate 3D positions and covariances of the detections //{ */
      for (const auto& pt : balloons.points)
      {
        const Eigen::Vector3d det_pos_sf(pt.x, pt.y, pt.z);
        const Eigen::Vector3d det_pos = s2w_tf*det_pos_sf;
        const Eigen::Matrix3d det_cov_sf = calc_position_covariance(det_pos_sf);
        const Eigen::Matrix3d det_cov = rotate_covariance(det_cov_sf, s2w_tf.rotation());
        if (det_cov.array().isNaN().any())
        {
          ROS_ERROR("Constructed covariance of detection [%.2f, %.2f, %.2f] contains NaNs!", det_pos_sf(0), det_pos_sf(1), det_pos_sf(2));
          ROS_ERROR("detection 3d position: [%.2f, %.2f, %.2f]", det_pos(0), det_pos(1), det_pos(2));
        }

        pos_cov_t pos_cov;
        pos_cov.position = det_pos;
        pos_cov.covariance = det_cov;
        pos_covs.push_back(pos_cov);
      }
      //}

      /* Process the LKFs - assign measurements and kick out too uncertain ones, find the most certain one //{ */
      {
        // the LKF must have received at least min_corrs_to_consider corrections
        // in order to be considered for the search of the most certain LKF
        int max_corrections = m_drmgr_ptr->config.min_corrs_to_consider;
        double picked_uncertainty;

        std::lock_guard<std::mutex> lck(m_lkfs_mtx);
        starting_lkfs = m_lkfs.size();
        if (!m_lkfs.empty())
        {
          for (std::list<Lkf>::iterator it = std::begin(m_lkfs); it != std::end(m_lkfs); it++)
          {
            auto& lkf = *it;

            /* Assign a measurement to the LKF based on the smallest divergence and update the LKF //{ */
            {
              double divergence;
              size_t closest_it = find_closest_measurement(lkf, pos_covs, divergence);

              /* Evaluate whether the divergence is small enough to justify the update //{ */
              if (divergence < m_drmgr_ptr->config.max_update_divergence)
              {
                Eigen::Vector3d closest_pos = pos_covs.at(closest_it).position;
                Eigen::Matrix3d closest_cov = pos_covs.at(closest_it).covariance;
                lkf.setMeasurement(closest_pos, closest_cov);
                lkf.doCorrection();
                meas_used.at(closest_it)++;
              }
              //}

            }
            //}

            /* Check if the uncertainty of this LKF is too high and if so, delete it, otherwise consider it for the most certain LKF //{ */
            {
              // First, check the uncertainty
              double uncertainty = calc_LKF_uncertainty(lkf);
              if (uncertainty > m_drmgr_ptr->config.max_lkf_uncertainty || std::isnan(uncertainty))
              {
                it = m_lkfs.erase(it);
                it--;
                kicked_out_lkfs++;
              } else
              // If LKF survived,  consider it as candidate to be picked.
              // The LKF is picked if it has higher number of corrections than the found maximum.
              // If it has the same number of corrections as a previously found maximum then uncertainties are
              // compared to decide which is going to be picked.
              if (
                  // current LKF has higher or equal number of corrections as is the current max. found
                  lkf.getNCorrections() >= max_corrections
                  // no LKF has been picker yet OR cur LKF has higher number of corrections OR cur LKF has same number of corrections but lower uncertainty
               && (most_certain_lkf == nullptr  || lkf.getNCorrections() > max_corrections  || uncertainty < picked_uncertainty)
                  )
              {
                most_certain_lkf = &lkf;
                max_corrections = lkf.getNCorrections();
                picked_uncertainty = uncertainty;
              }
            }
            //}

          }
        }
        if (most_certain_lkf != nullptr)
          std::cout << "Most certain LKF found with " << picked_uncertainty << " uncertainty " << " and " << most_certain_lkf->getNCorrections() << " correction iterations" << std::endl;
      }
      //}

      /* Instantiate new LKFs for unused measurements (these are not considered as candidates for the most certain LKF) //{ */
      {
        std::lock_guard<std::mutex> lck(m_lkfs_mtx);
        for (size_t it = 0; it < n_meas; it++)
        {
          if (meas_used.at(it) < 1)
          {
            create_new_lkf(m_lkfs, pos_covs.at(it));
            new_lkfs++;
          }
        }
        ending_lkfs = m_lkfs.size();
      }
      //}

    }
    //}

    /* Publish message of the most likely LKF (if found) //{ */
    if (most_certain_lkf != nullptr)
    {
      std::cout << "Publishing most certain LKF result from LKF#" << most_certain_lkf->id << std::endl;
      geometry_msgs::PoseWithCovarianceStamped msg = create_message(*most_certain_lkf, balloons.header.stamp);
      m_pub_odom_balloon.publish(msg);
    } else
    {
      std::cout << "No LKF is certain yet, publishing nothing" << std::endl;
    }
    //}

    std::cout << "LKFs: " << starting_lkfs << " - " << kicked_out_lkfs << " + " << new_lkfs << " = " << ending_lkfs << std::endl;
    ros::Time end_t = ros::Time::now();
    static double dt = (end_t - start_t).toSec();
    dt = 0.9 * dt + 0.1 * (end_t - start_t).toSec();
    std::cout << "processing FPS: " << 1 / dt << "Hz" << std::endl;

    ROS_INFO("[%s]: New data processed", m_node_name.c_str());
  }
}

/* onInit() //{ */

void BalloonPlanner::onInit()
{

  ROS_INFO("[%s]: Initializing", m_node_name.c_str());
  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();
  ros::Time::waitForValid();

  /* initialize variables //{ */

  m_last_lkf_id = 0;

  //}

  /* load parameters //{ */

  mrs_lib::ParamLoader pl(nh, m_node_name);

  double planning_period = pl.load_param2<double>("planning_period");
  double lkf_dt = pl.load_param2<double>("lkf_dt", lkf_dt);
  pl.load_param("world_frame", m_world_frame, std::string("local_origin"));

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

  m_tf_listener_ptr = std::make_unique<tf2_ros::TransformListener>(m_tf_buffer);
  mrs_lib::SubscribeMgr smgr(nh);
  m_sh_balloons = smgr.create_handler_threadsafe<sensor_msgs::PointCloud>("balloon_cloud_in", 10, ros::TransportHints().tcpNoDelay(), ros::Duration(5.0));

  if (!smgr.loaded_successfully())
  {
    ROS_ERROR("Unable to subscribe to some topics, ending the node");
    ros::shutdown();
  }

  //}

  /* publishers //{ */

  m_pub_odom_balloon = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("balloon_odom_out", 1);

  //}

  /* profiler //{ */

  m_profiler_ptr = std::make_unique<mrs_lib::Profiler>(nh, m_node_name, false);

  //}

  m_is_initialized = true;

  /* timers  //{ */

  m_main_loop_timer = nh.createTimer(ros::Duration(planning_period), &BalloonPlanner::main_loop, this);
  m_lkf_update_timer = nh.createTimer(ros::Duration(lkf_dt), &BalloonPlanner::lkf_update, this);

  //}

  ROS_INFO("[%s]: initialized", m_node_name.c_str());
}

//}

}  // namespace balloon_planner

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(balloon_planner::BalloonPlanner, nodelet::Nodelet)
