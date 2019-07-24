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
    x_yaw,   // yaw of the MAV in the eight-plane
    x_s,     // the ball speed
    x_c,     // curvature of the MAV trajectory in the eight-plane
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
  using Vec2 = Eigen::Vector2d;

  // from https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
  template <typename T>
  T sign(T val)
  {
    return (T(0) < val) - (val < T(0));
  }

  UKF::x_t tra_model_f(const UKF::x_t& in, [[maybe_unused]] const UKF::u_t& u, const double dt)
  {
    x_t out;

    // Calculate the complete current state with redundant variables
    const double yaw = in(x_yaw);
    const double speed = std::max(in(x_s), 0.0);
    const double curv = in(x_c);
    // reference for curvature: https://web.ma.utexas.edu/users/m408m/Display13-4-3.shtml
    const Quat quat = Quat(in(x_qw), in(x_qx), in(x_qy), in(x_qz)).normalized();
    const Vec3 tang(cos(yaw), sin(yaw), 0.0);
    const Vec3 norm(cos(yaw + M_PI_2), sin(yaw + M_PI_2), 0.0);
    const Vec3 vel_tang = speed*tang;
    const Vec3 acc_norm = curv*speed*speed*norm;
    const Vec3 dpos = vel_tang*dt + 0.5*acc_norm*dt*dt;
    const Vec3 pos_world(in(x_x), in(x_y), in(x_z));

    // Calculate the next estimated state
    const Vec3 n_pos_world = pos_world + quat*dpos;
    const double n_speed = speed; // assume constant speed
    /* const double n_yaw = yaw; */
    const double n_yaw = yaw + speed*curv*dt;
    const double n_curv = curv;
    const Quat n_quat = quat; // does not change

    // Copy the calculated values to the respective states
    out(x_x) = n_pos_world.x();
    out(x_y) = n_pos_world.y();
    out(x_z) = n_pos_world.z();
    out(x_yaw) = n_yaw;
    out(x_s) = n_speed;
    out(x_c) = n_curv;
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

  double normalize_angle(double in)
  {
    double out = std::fmod(in, 2*M_PI);
    if (out > M_PI)
      out -= 2*M_PI;
    else if (out < -M_PI)
      out += 2*M_PI;
    return out;
  }

  UKF::x_t normalize_state(const UKF::x_t& in)
  {
    UKF::x_t out = in;
    out(x_yaw) = normalize_angle(in(x_yaw));
    // TODO: normalize the quaternion as well?
    return out;
  }
}



#include <iostream>
#include <fstream>

using namespace balloon_planner;

/* load_csv() function //{ */
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
//}

/* save_csv() function //{ */
template<typename M>
void save_csv(const M& mat, const std::string& path, const char delim = ',', const std::string header = std::string())
{
  std::ofstream odata(path);
  if (!odata.is_open())
  {
    std::cerr << "Could not open " << path << std::endl;
    return;
  }
  if (!header.empty())
    odata << header << std::endl;
  for (int row = 0; row < mat.rows(); row++)
  {
    for (int col = 0; col < mat.cols(); col++)
    {
      odata << mat(row, col);
      if (col != mat.cols() -1)
        odata << delim;
    }
    odata << std::endl;
  }
}
//}

int main()
{
  std::string fname = "data.csv";

  const char delim = ',';
  const Eigen::MatrixXd pts = load_csv<Eigen::MatrixXd>(fname, delim, true);
  const int n_pts = pts.rows();
  const int cols = pts.cols();
  std::cout << "Loaded " << n_pts << " points" << std::endl;

  const double dt = 1.0;
  Q_t Q = Q_t::Identity();
  Q.block<3, 3>(0, 0) *= 1e0;
  Q(3, 3) *= 1e-2;
  Q(4, 4) *= 1e-1;
  Q(5, 5) *= 5e-1;
  /* Q.block<4, 4>(6, 6) *= 1e-1; */
  tra_model_t tra_model(tra_model_f);
  obs_model_t obs_model(obs_model_f);

  UKF ukf(1e-3, 1, 2, Q, tra_model, obs_model);

  const double R_std = 1e0;
  R_t R = R_std*R_t::Identity();
  const x_t x0((x_t() << 
      0,
      0,
      0,
      0.5,
      0.5,
      0,
      1,
      0,
      0,
      1
      ).finished()
      );
  const P_t P0 = 1e2*Q;
  double err_acc = 0.0;
  const int err_states = std::min(cols, kf_n_states);
  statecov_t sc {x0, P0};
  Eigen::MatrixXd est_states(n_pts, kf_n_states);
  for (int it = 0; it < n_pts; it++)
  {
    std::cout << "iteration " << it << std::endl;
    const auto cur_gt = pts.block(it, 0, 1, err_states).transpose();
    const auto cur_pt = pts.block<1, 3>(it, 0).transpose();
    std::cout << "state ground-truth: " << std::endl << cur_gt.transpose() << std::endl;

    std::cout << "ukf state: " << std::endl << sc.x.transpose() << std::endl;
    sc = ukf.predict(sc, u_t(), dt);
    sc.x = normalize_state(sc.x);
    std::cout << "state prediction: " << std::endl << sc.x.transpose() << std::endl;
    sc = ukf.correct(sc, cur_pt, R);
    sc.x = normalize_state(sc.x);
    const auto cur_est = sc.x.block(0, 0, err_states, 1);
    const double cur_err = (cur_gt-cur_est).norm();
    err_acc += cur_err;
    std::cout << "state correction: " << std::endl << sc.x.transpose() << std::endl;
    est_states.block<1, kf_n_states>(it, 0) = sc.x.transpose();

    /* std::cout << "state covariance: " << std::endl << sc.P << std::endl; */
    std::cout << "current error: " << std::endl << cur_err << std::endl;
    std::cout << "accumulated error: " << std::endl << err_acc << std::endl;
  }

  save_csv(est_states, "est.csv", ',', "x,y,z,yaw,speed,curvature,qw,qx,qy,qz");
}
