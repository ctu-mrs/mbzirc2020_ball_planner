#include <balloon_planner/eight_ukf.h>

using namespace balloon_planner;

using x_t = UKF::x_t;
using P_t = UKF::P_t;
using u_t = UKF::u_t;
using z_t = UKF::z_t;
using R_t = UKF::R_t;
using statecov_t = UKF::statecov_t;

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
  u_x = 0, // 3D x-coordinate of the ball position
  u_y,     // 3D y-coordinate of the ball position
  u_z,     // 3D z-coordinate of the ball position
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

x_t tra_model_f(const x_t& in, const u_t& u, const double dt)
{
  x_t out;

  // Calculate the complete current state with redundant variables
  const double speed = in(x_s);
  const double yaw = in(x_yaw);
  const Quat quat = Quat(in(x_qw), in(x_qx), in(x_qy), in(x_qz)).normalized();
  const Vec3 vel_eight(speed*cos(yaw), speed*sin(yaw), 0.0);
  const Vec3 vel_world = quat * vel_eight;
  const Vec3 pos_world(in(x_x), in(x_y), in(x_z));

  // Calculate the next estimated state
  const Vec3 n_pos_world = pos_world + vel_world*dt; // TODO: take into account the curvature as well!
  const Quat n_quat = quat; // does not change


  return out;
}

// This function implements the observation generation from a state
z_t obs_model_f(const x_t& state)
{
  z_t observation;
  observation(z_x) = state(x_x);
  observation(z_y) = state(x_y);
  observation(z_z) = state(x_z);
  return observation;
}
