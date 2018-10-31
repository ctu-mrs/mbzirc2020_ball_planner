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

// Geometry msgs
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

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
#include <LkfAssociation.h>

//}

namespace balloon_planner
{
  // shortcut type to the dynamic reconfigure manager template instance
  typedef mrs_lib::DynamicReconfigureMgr<balloon_planner::PlanningParamsConfig> drmgr_t;
  using Lkf = balloon_planner::LkfAssociation;

  /* //{ class BalloonPlanner */

  class BalloonPlanner : public nodelet::Nodelet
  {

    private:

      /* pos_cov_t helper struct //{ */
      struct pos_cov_t
      {
        Eigen::Vector3d position;
        Eigen::Matrix3d covariance;
      };
      //}
    
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
      //}

      /* ROS related variables (subscribers, timers etc.) //{ */
      std::unique_ptr<drmgr_t> m_drmgr_ptr;
      tf2_ros::Buffer m_tf_buffer;
      std::unique_ptr<tf2_ros::TransformListener> m_tf_listener_ptr;
      mrs_lib::SubscribeHandlerPtr<sensor_msgs::PointCloud> m_sh_balloons;
      ros::Publisher m_pub_odom_balloon;
      ros::Timer m_lkf_update_timer;
      ros::Timer m_main_loop_timer;
      //}

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
          geometry_msgs::TransformStamped transform;
          // Obtain transform from snesor into world frame
          transform = m_tf_buffer.lookupTransform(m_world_frame, frame_name, stamp, timeout);

          // Obtain transform from camera frame into world
          tf_out = tf2::transformToEigen(transform.transform);
        } catch (tf2::TransformException& ex)
        {
          ROS_WARN("Error during transform from \"%s\" frame to \"%s\" frame.\n\tMSG: %s", frame_name.c_str(), m_world_frame.c_str(), ex.what());
          return false;
        }
        return true;
      }
      //}

      /* calc_position_covariance() method //{ */
      /* position_sf is position of the detection in 3D in the frame of the sensor (camera) */
      Eigen::Matrix3d calc_position_covariance(const Eigen::Vector3d& position_sf)
      {
        /* Calculates the corresponding covariance matrix of the estimated 3D position */
        Eigen::Matrix3d pos_cov = Eigen::Matrix3d::Identity();  // prepare the covariance matrix
        const double tol = 1e-9;
        pos_cov(0, 0) = pos_cov(1, 1) = m_drmgr_ptr->config.xy_covariance_coeff;

        pos_cov(2, 2) = position_sf(2)*sqrt(position_sf(2))*m_drmgr_ptr->config.z_covariance_coeff;
        if (pos_cov(2, 2) < 0.33*m_drmgr_ptr->config.z_covariance_coeff)
          pos_cov(2, 2) = 0.33*m_drmgr_ptr->config.z_covariance_coeff;

        // Find the rotation matrix to rotate the covariance to point in the direction of the estimated position
        const Eigen::Vector3d a(0.0, 0.0, 1.0);
        const Eigen::Vector3d b = position_sf.normalized();
        const Eigen::Vector3d v = a.cross(b);
        const double sin_ab = v.norm();
        const double cos_ab = a.dot(b);
        Eigen::Matrix3d vec_rot = Eigen::Matrix3d::Identity();
        if (sin_ab < tol)  // unprobable, but possible - then it is identity or 180deg
        {
          if (cos_ab + 1.0 < tol)  // that would be 180deg
          {
            vec_rot << -1.0, 0.0, 0.0,
                       0.0, -1.0, 0.0,
                       0.0, 0.0, 1.0;
          } // otherwise its identity
        } else  // otherwise just construct the matrix
        {
          Eigen::Matrix3d v_x; v_x << 0.0, -v(2), v(1),
                                      v(2), 0.0, -v(0),
                                      -v(1), v(0), 0.0;
          vec_rot = Eigen::Matrix3d::Identity() + v_x + (1-cos_ab)/(sin_ab*sin_ab)*(v_x*v_x);
        }
        pos_cov = rotate_covariance(pos_cov, vec_rot);  // rotate the covariance to point in direction of est. position
        return pos_cov;
      }
      //}

      /* rotate_covariance() method //{ */
      Eigen::Matrix3d rotate_covariance(const Eigen::Matrix3d& covariance, const Eigen::Matrix3d& rotation)
      {
        return rotation*covariance*rotation.transpose();  // rotate the covariance to point in direction of est. position
      }
      //}

      /* calc_LKF_uncertainty() method //{ */
      double calc_LKF_uncertainty(const Lkf& lkf)
      {
        Eigen::Matrix3d position_covariance = lkf.getCovariance().block<3, 3>(0, 0);
        double determinant = position_covariance.determinant();
        return sqrt(determinant);
      }
      //}

      /* calc_LKF_meas_divergence() method //{ */
      double calc_LKF_meas_divergence(const Eigen::Vector3d& mu0, const Eigen::Matrix3d& sigma0, const Eigen::Vector3d& mu1, const Eigen::Matrix3d& sigma1)
      {
        return kullback_leibler_divergence(mu0, sigma0, mu1, sigma1);
      }
      //}

      /* find_closest_measurement() method //{ */
      /* returns position of the closest measurement in the pos_covs vector */
      size_t find_closest_measurement(const Lkf& lkf, const std::vector<pos_cov_t>& pos_covs, double& min_divergence_out)
      {
        const Eigen::Vector3d& lkf_pos = lkf.getStates().block<3, 1>(0, 0);
        const Eigen::Matrix3d& lkf_cov = lkf.getCovariance().block<3, 3>(0, 0);
        double min_divergence = std::numeric_limits<double>::max();
        size_t min_div_it = 0;

        // Find measurement with smallest divergence from this LKF and assign the measurement to it
        for (size_t it = 0; it < pos_covs.size(); it++)
        {
          const auto& pos_cov = pos_covs.at(it);
          if (pos_cov.covariance.array().isNaN().any())
            ROS_ERROR("Covariance of LKF contains NaNs!");
          const Eigen::Vector3d& det_pos = pos_cov.position;
          const Eigen::Matrix3d& det_cov = pos_cov.covariance;
          const double divergence = calc_LKF_meas_divergence(det_pos, det_cov, lkf_pos, lkf_cov);
        
          if (divergence < min_divergence)
          {
            min_divergence = divergence;
            min_div_it = it;
          }
        }
        min_divergence_out = min_divergence;
        return min_div_it;
      }
      //}

      /* create_message() method //{ */
      geometry_msgs::PoseWithCovarianceStamped create_message(const Lkf& lkf, ros::Time stamp)
      {
        geometry_msgs::PoseWithCovarianceStamped msg;

        msg.header.frame_id = m_world_frame;
        msg.header.stamp = stamp;

        {
          const Eigen::Vector3d position = lkf.getStates().block<3, 1>(0, 0);
          msg.pose.pose.position.x = position(0);
          msg.pose.pose.position.y = position(1);
          msg.pose.pose.position.z = position(2);
        }

        msg.pose.pose.orientation.w = 1.0;

        {
          const Eigen::Matrix3d covariance = lkf.getCovariance().block<3, 3>(0, 0);
          for (int r = 0; r < 6; r++)
          {
            for (int c = 0; c < 6; c++)
            {
              if (r < 3 && c < 3)
                msg.pose.covariance[r*6 + c] = covariance(r, c);
              else if (r == c)
                msg.pose.covariance[r*6 + c] = 666;
            }
          }
        }

        return msg;
      }
      //}

    private:

      /* LKF - related member variables //{ */
      std::mutex m_lkfs_mtx; // mutex for synchronization of the m_lkfs variable
      std::list<Lkf> m_lkfs; // all currently active LKFs
      int m_last_lkf_id; // ID of the last created LKF - used when creating a new LKF to generate a new unique ID
      //}
      
      /* Definitions of the LKF (consts, typedefs, etc.) //{ */
      static const int c_n_states = 3;
      static const int c_n_inputs = 0;
      static const int c_n_measurements = 3;

      typedef Eigen::Matrix<double, c_n_states, 1> lkf_x_t;
      typedef Eigen::Matrix<double, c_n_inputs, 1> lkf_u_t;
      typedef Eigen::Matrix<double, c_n_measurements, 1> lkf_z_t;

      typedef Eigen::Matrix<double, c_n_states, c_n_states> lkf_A_t;
      typedef Eigen::Matrix<double, c_n_states, c_n_inputs> lkf_B_t;
      typedef Eigen::Matrix<double, c_n_measurements, c_n_states> lkf_P_t;
      typedef Eigen::Matrix<double, c_n_states, c_n_states> lkf_R_t;
      typedef Eigen::Matrix<double, c_n_measurements, c_n_measurements> lkf_Q_t;

      lkf_A_t create_A(double dt)
      {
        lkf_A_t A;
        A << 1, 0, 0,
             0, 1, 0,
             0, 0, 1;
        return A;
      }

      lkf_P_t create_P()
      {
        lkf_P_t P;
        P << 1, 0, 0,
             0, 1, 0,
             0, 0, 1;
        return P;
      }

      lkf_R_t create_R(double dt)
      {
        lkf_R_t R = dt*m_drmgr_ptr->config.lkf_process_noise_pos*lkf_R_t::Identity();
        return R;
      }
      //}

      /* create_new_lkf() method //{ */
      void create_new_lkf(std::list<Lkf>& lkfs, pos_cov_t& initialization)
      {
        const lkf_A_t A; // changes in dependence on the measured dt, so leave blank for now
        const lkf_B_t B; // zero rows zero cols matrix
        const lkf_P_t P = create_P();
        const lkf_R_t R; // depends on the measured dt, so leave blank for now
        const lkf_Q_t Q; // depends on the measurement, so leave blank for now
      
        lkfs.emplace_back(m_last_lkf_id, c_n_states, c_n_inputs, c_n_measurements, A, B, R, Q, P);
        m_last_lkf_id++;
        Lkf& new_lkf = lkfs.back();

        // Initialize the LKF using the new measurement
        lkf_x_t init_state = initialization.position;
        lkf_R_t init_state_cov = initialization.covariance;

        new_lkf.setStates(init_state);
        new_lkf.setCovariance(init_state_cov);
      }
      //}

      /* lkf_update() method //{ */
      void lkf_update(const ros::TimerEvent& evt)
      {
        double dt = (evt.current_real - evt.last_real).toSec();
        lkf_A_t A = create_A(dt);
        lkf_R_t R = create_R(dt);

        {
          std::lock_guard<std::mutex> lck(m_lkfs_mtx);
          for (auto& lkf : m_lkfs)
          {
            lkf.setA(A);
            lkf.setR(R);
            lkf.iterateWithoutCorrection();
          }
        }
      }
      //}

    private:

      // --------------------------------------------------------------
      // |        detail implementation methods (maybe unused)        |
      // --------------------------------------------------------------

      /* kullback_leibler_divergence() method //{ */
      // This method calculates the kullback-leibler divergence of two three-dimensional normal distributions.
      // It is used for deciding which measurement to use for which LKF.
      double kullback_leibler_divergence(const Eigen::Vector3d& mu0, const Eigen::Matrix3d& sigma0, const Eigen::Vector3d& mu1, const Eigen::Matrix3d& sigma1)
      {
        const unsigned k = 3; // number of dimensions -- DON'T FORGET TO CHANGE IF NUMBER OF DIMENSIONS CHANGES!
        const double div = 0.5*( (sigma1.inverse()*sigma0).trace() + (mu1-mu0).transpose()*(sigma1.inverse())*(mu1-mu0) - k + log((sigma1.determinant())/sigma0.determinant()));
        return div;
      }
      //}

  };
  
  //}

}  // namespace balloon_planner

#endif
