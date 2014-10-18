#ifndef GAUSS_SOLVER_H
#define GAUSS_SOLVER_H

#include <Eigen/Dense>

namespace zsw{

  /**
   * return 0 if solve success.
   */
  int solve(const Eigen::MatrixXd &A, const Eigen::VectorXd &B, Eigen::VectorXd &x);
}

#endif /* GAUSS_SOLVER_H */
