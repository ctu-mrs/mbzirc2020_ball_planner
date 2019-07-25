#include <mrs_lib/ukf.h>

namespace balloon_planner
{
  /* constexpr int kf_n_states = 10; */
  constexpr int kf_n_states = 10;
  constexpr int kf_n_inputs = 0;
  constexpr int kf_n_measurements = 3;
  using UKF = mrs_lib::UKF<kf_n_states, kf_n_inputs, kf_n_measurements>;

  // This function implements the state transition
  UKF::x_t tra_model_f(const UKF::x_t& x, [[maybe_unused]] const UKF::u_t& u, const double dt);

  // This function implements the observation generation from a state
  UKF::z_t obs_model_f(const UKF::x_t& x);

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
}
