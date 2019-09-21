#include <balloon_planner/eight_rheiv.h>

template class mrs_lib::RHEIV<balloon_planner::rheiv::n_states, balloon_planner::rheiv::n_params>;

namespace balloon_planner
{
  namespace rheiv
  {
    using x_t = RHEIV::x_t;
    using xs_t = RHEIV::xs_t;

    using P_t = RHEIV::P_t;
    using Ps_t = RHEIV::Ps_t;

    using z_t = RHEIV::z_t;
    using zs_t = RHEIV::zs_t;

    using f_z_t = RHEIV::f_z_t;
    using dzdx_t = RHEIV::dzdx_t;

    using Quat = Eigen::Quaterniond;
    using Vec3 = Eigen::Vector3d;
    using Vec2 = Eigen::Vector2d;

    /* For the plane surface model, there is no need to transform the data. */
    zs_t f_z_f(const xs_t & xs)
    {
      return xs;
    }
  }
}

#ifdef COMPILE_EIGHT_RHEIV_TEST

#include <iostream>
#include <fstream>
#include <random>

using namespace balloon_planner;
using namespace balloon_planner::rheiv;

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

/* normal_randvec() function //{ */
template <int rows>
Eigen::Matrix<double, rows, 1> normal_randvec(const Eigen::Matrix<double, rows, rows>& cov)
{
    static std::random_device rd{};
    static std::mt19937 gen{rd()};
    static std::normal_distribution<> d{0,1};
    Eigen::Matrix<double, rows, 1> ret;
    for (int row = 0; row < rows; row++)
      ret(row, 0) = d(gen);
    return cov*ret;
}
//}

/* main() function for testing //{ */
int main()
{
  std::string fname = "/home/matous/balloon_workspace/src/ros_packages/balloon_planner/data.csv";

  const char delim = ',';
  const Eigen::MatrixXd pts = load_csv<Eigen::MatrixXd>(fname, delim, true);
  const int n_pts = pts.rows();
  const int cols = pts.cols();
  assert(cols >= n_states);
  std::cout << "Loaded " << n_pts << " points" << std::endl;

  const f_z_t f_z(f_z_f);
  const dzdx_t dzdx = dzdx_t::Identity();

  RHEIV rheiv(f_z, dzdx);

  xs_t xs(n_states, n_pts);
  Ps_t Ps(n_pts);
  for (int it = 0; it < n_pts; it++)
  {
    const auto cur_pt = pts.block<1, n_states>(it, 0).transpose();
    const P_t tmp = P_t::Random();
    const P_t P = tmp*tmp.transpose();
    const x_t cur_x = cur_pt + normal_randvec(P);
    xs.block(0, it, n_states, 1) = cur_x;
    Ps.at(it) = P;
  }

  const auto theta = rheiv.fit(xs, Ps);
  std::cout << "theta (norm: " << theta.norm() << "):" << std::endl << theta << std::endl;

  save_csv(theta, "theta.csv", ',', "a,b,c,d");
}
//}

#endif // COMPILE_EIGHT_UKF_TEST

