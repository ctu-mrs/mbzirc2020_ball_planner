#include <mrs_lib/ukf.h>

namespace balloon_planner
{
  constexpr int kf_n_states = 10;
  constexpr int kf_n_inputs = 0;
  constexpr int kf_n_measurements = 3;
  using UKF = mrs_lib::UKF<kf_n_states, kf_n_inputs, kf_n_measurements>;

  // This function implements the state transition
  UKF::x_t tra_model_f(const UKF::x_t& x, [[maybe_unused]] const UKF::u_t& u, const double dt);

  // This function implements the observation generation from a state
  UKF::z_t obs_model_f(const UKF::x_t& x);
}
