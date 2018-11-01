#include <Eigen/Dense>
#include "mrs_lib/Lkf.h"

namespace balloon_planner
{
  class LkfAssociation
  {
    public:
      LkfAssociation()
        : id(-1), lkf_impl(0, 0, 0, Eigen::MatrixXd(), Eigen::MatrixXd(), Eigen::MatrixXd(), Eigen::MatrixXd(), Eigen::MatrixXd())
      {};

      LkfAssociation(const LkfAssociation& lkf)
        : id(lkf.id), m_n_corrections(lkf.m_n_corrections), lkf_impl(lkf.lkf_impl)
      {};

      LkfAssociation(const int id,
                const int n, const int m, const int p,
                const Eigen::MatrixXd A, const Eigen::MatrixXd B,
                const Eigen::MatrixXd R, const Eigen::MatrixXd Q,
                const Eigen::MatrixXd P)
        : id(id), m_n_corrections(0), lkf_impl(n, m, p, A, B, R, Q, P)
      {};

      LkfAssociation operator=(const LkfAssociation& o)
      {
        return LkfAssociation(o);
      };

      const int id;

    public:

      Eigen::VectorXd getStates(void) const
      {
        return lkf_impl.getStates();
      }
  
      void setMeasurement(const Eigen::VectorXd& newMes, const Eigen::MatrixXd& newCov)
      {
        return lkf_impl.setMeasurement(newMes, newCov);
      }

      Eigen::MatrixXd getCovariance(void) const
      {
        return lkf_impl.getCovariance();
      }

      void setA(const Eigen::MatrixXd& A)
      {
        lkf_impl.setA(A);
      }

      void setP(const Eigen::MatrixXd& P)
      {
        lkf_impl.setP(P);
      }

      void setR(const Eigen::MatrixXd& R)
      {
        lkf_impl.setR(R);
      }

      void setStates(const Eigen::VectorXd& states)
      {
        lkf_impl.setStates(states);
      }

      void setCovariance(const Eigen::MatrixXd& cov)
      {
        lkf_impl.setCovariance(cov);
      }

      Eigen::VectorXd iterateWithoutCorrection(void)
      {
        return lkf_impl.iterateWithoutCorrection();
      }

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
