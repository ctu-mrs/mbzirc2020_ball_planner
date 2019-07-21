#include <balloon_planner/eight_ukf.h>

namespace balloon_planner
{

  using x_t = UKF::x_t;
  using P_t = UKF::P_t;
  using u_t = UKF::u_t;
  using z_t = UKF::z_t;
  using Q_t = UKF::Q_t;
  using R_t = UKF::R_t;
  using statecov_t = UKF::statecov_t;
  using tra_model_t = UKF::transition_model_t;
  using obs_model_t = UKF::observation_model_t;

  // indices of the state interpretations
  enum
  {
    x_x = 0, // 3D x-coordinate of the ball position
    x_y,     // 3D y-coordinate of the ball position
    x_z,     // 3D z-coordinate of the ball position
    x_s,     // the ball speed
    x_yaw,   // yaw of the MAV in the eight-plane
    x_c,     // curvature of the MAV trajectory in the eight-plane
    x_dc,    // derivative of curvature of the MAV trajectory in the eight-plane
    x_qw,    // w element of quaterion, defining rotation from world frame to the eight-plane frame
    x_qx,    // x element of quaterion, defining rotation from world frame to the eight-plane frame
    x_qy,    // y element of quaterion, defining rotation from world frame to the eight-plane frame
    x_qz,    // z element of quaterion, defining rotation from world frame to the eight-plane frame
  };

  // indices of the measurement interpretations
  enum
  {
    z_x = 0, // 3D x-coordinate of the ball position
    z_y,     // 3D y-coordinate of the ball position
    z_z,     // 3D z-coordinate of the ball position
  };

  using Quat = Eigen::Quaterniond;
  using Vec3 = Eigen::Vector3d;

  UKF::x_t tra_model_f(const UKF::x_t& in, [[maybe_unused]] const UKF::u_t& u, const double dt)
  {
    x_t out;

    // Calculate the complete current state with redundant variables
    const double speed = in(x_s);
    const double yaw = in(x_yaw);
    const double curv = in(x_c);
    const double dcurv = in(x_dc);
    const double ang_speed = curv*speed;
    const Quat quat = Quat(in(x_qw), in(x_qx), in(x_qy), in(x_qz)).normalized();
    const Vec3 vel_eight(speed*cos(yaw), speed*sin(yaw), 0.0);
    const Vec3 vel_world = quat * vel_eight;
    const Vec3 pos_world(in(x_x), in(x_y), in(x_z));

    // Calculate the next estimated state
    const Vec3 n_pos_world = pos_world + vel_world*dt; // TODO: take into account the curvature as well!
    const double n_speed = speed; // assume constant speed
    const double n_yaw = yaw + ang_speed*dt;
    const double n_curv = curv + dcurv*dt;
    const double n_dcurv = dcurv; // no model of a curvature derivative changing
    const Quat n_quat = quat; // does not change

    // Copy the calculated values to the respective states
    out(x_x) = n_pos_world.x();
    out(x_y) = n_pos_world.y();
    out(x_z) = n_pos_world.z();
    out(x_s) = n_speed;
    out(x_yaw) = n_yaw;
    out(x_c) = n_curv;
    out(x_dc) = n_dcurv;
    out(x_qw) = n_quat.w();
    out(x_qx) = n_quat.x();
    out(x_qy) = n_quat.y();
    out(x_qz) = n_quat.z();

    return out;
  }

  // This function implements the observation generation from a state
  UKF::z_t obs_model_f(const UKF::x_t& state)
  {
    z_t observation;
    observation(z_x) = state(x_x);
    observation(z_y) = state(x_y);
    observation(z_z) = state(x_z);
    return observation;
  }

}
#include <iostream>
#include <fstream>

using namespace balloon_planner;

// from https://stackoverflow.com/questions/34247057/how-to-read-csv-file-and-assign-to-eigen-matrix
template<typename M>
M load_csv(const std::string& path, const char delim = ',', bool skip_header = false)
{
  std::ifstream indata(path);
  if (!indata.is_open())
  {
    std::cerr << "Could not open " << path << std::endl;
    return M();
  }
  std::string line;
  std::vector<double> values;
  int rows = 0;
  while (std::getline(indata, line))
  {
    if (skip_header)
    {
      skip_header = false;
      continue;
    }
    std::stringstream line_stream(line);
    std::string cell;
    while (std::getline(line_stream, cell, delim))
    {
      values.push_back(std::stod(cell));
    }
    rows++;
  }
  return Eigen::Map<const Eigen::Matrix<typename M::Scalar, M::RowsAtCompileTime, M::ColsAtCompileTime, Eigen::RowMajor>>(values.data(), rows, values.size()/rows);
}

int main()
{
  std::string fname = "data.csv";

  const char delim = ',';
  const Eigen::MatrixXd pts = load_csv<Eigen::MatrixXd>(fname, delim, true);
  const int n_pts = pts.rows();
  std::cout << "Loaded " << n_pts << " points" << std::endl;

  Q_t Q = Q_t::Identity();
  Q.block<3, 3>(0, 0) *= 1e0;
  Q(3, 3) *= 2e-1;
  Q(4, 4) *= 1e-1;
  Q(5, 5) *= 5e-1;
  Q(6, 6) *= 1e-1;
  Q.block<4, 4>(7, 7) *= 1e-1;
  tra_model_t tra_model(tra_model_f);
  obs_model_t obs_model(obs_model_f);

  UKF ukf(1e-3, 1, 2, Q, tra_model, obs_model);

  const double R_std = 3.0;
  R_t R = R_std*R_t::Identity();
  const x_t x0((x_t() << 
      0,
      0,
      0,
      0.5,
      0,
      0,
      0,
      1,
      0,
      0,
      0
      ).finished()
      );
  const P_t P0 = 1e2*Q;
  statecov_t sc {x0, P0};
  for (int it = 0; it < n_pts; it++)
  {
    std::cout << "iteration " << it << std::endl;
    const auto cur_pt = pts.block<1, 3>(it, 0).transpose();
    sc = ukf.predict(sc, u_t(), 1.0);
    sc = ukf.correct(sc, cur_pt, R);
    std::cout << "state estimate: " << std::endl << sc.x.transpose() << std::endl;
    std::cout << "state covariance: " << std::endl << sc.P << std::endl;
    std::cout << "state ground-truth: " << std::endl << cur_pt.transpose() << std::endl;
  }
}
