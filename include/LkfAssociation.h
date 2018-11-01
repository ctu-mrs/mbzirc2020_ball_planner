#include <Eigen/Dense>
#include "mrs_lib/Lkf.h"

namespace balloon_planner
{
  class LkfAssociation
  {
    public:
      LkfAssociation()
        : id(-1)
      {};

      LkfAssociation(const LkfAssociation& lkf)
        : Lkf(lkf), id(id), m_n_corrections(0)
      {};

      LkfAssociation(const int id,
                const int n, const int m, const int p,
                const Eigen::MatrixXd A, const Eigen::MatrixXd B,
                const Eigen::MatrixXd R, const Eigen::MatrixXd Q,
                const Eigen::MatrixXd P)
        : id(id), m_n_corrections(0), lkf_impl(n, m, p, A, B, R, Q, P)
      {};

      const int id;

      Eigen::VectorXd doCorrection(void)
      {
        m_n_corrections++;
        return lkf_impl.doCorrection();
      }

      int getNCorrections(void) const
      {
        return m_n_corrections;
      }

    private:
      int m_n_corrections;
      mrs_lib::Lkf lkf_impl;

  };
}
