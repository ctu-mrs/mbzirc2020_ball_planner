#include <balloon_planner/BalloonPlanner.h>
#include <balloon_planner/Lkf.h>

namespace balloon_planner
{
  template class Lkf_base<lkf_n_states, lkf_n_inputs, lkf_n_measurements>;
}  // namespace balloon_planner
