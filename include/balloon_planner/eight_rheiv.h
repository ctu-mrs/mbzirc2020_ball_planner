#include <mrs_lib/rheiv.h>

namespace balloon_planner
{
  namespace rheiv
  {
    constexpr int n_states = 3;
    constexpr int n_params = 4;
    using RHEIV = mrs_lib::RHEIV<n_states, n_params>;

    // This function implements the state transition
    RHEIV::zs_t f_z_f(const RHEIV::xs_t& xs);

    // indices of the state interpretations
    // the state represents plane parameters a*x + b*y + c*z + d = 0
    enum
    {
      x_a = 0, // 3D x-coordinate of the ball position
      x_b,     // 3D y-coordinate of the ball position
      x_c,     // 3D z-coordinate of the ball position
      x_d,     // yaw of the MAV in the eight-plane
    };
  }
}
