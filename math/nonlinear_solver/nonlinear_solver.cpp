#include "nonlinear_solver.h"

#include <assert.h>
#include <Eigen/LU>

#include <error/error.h>


int zsw::solve(const pEquationsFunc eq_func, const int max_iter, const double err,
            Eigen::VectorXd &x)
{
  std::size_t dim = eq_func->dim();
  assert(x.size() == dim);

  Eigen::VectorXd next_x = x;
  Eigen::MatrixXd jac_mat(dim, dim);
  Eigen::MatrixXd inv_jac_mat(dim,dim);
  Eigen::VectorXd val(dim);
  int ret_code = 0;
  for (int itn=0; itn<max_iter; ++itn) {
    ret_code = eq_func->jac(x, jac_mat);
    CHECK_RETCODE(ret_code);
    std::cout << "jac:" << std::endl;
    std::cout << jac_mat << std::endl;
    bool invertable = false;
    Eigen::FullPivLU<Eigen::MatrixXd> lu(jac_mat);
    if (!lu.isInvertible()) { return -1; }
    inv_jac_mat = jac_mat.inverse();

    ret_code = eq_func->val(x, val);
    CHECK_RETCODE(ret_code);
    std::cout << "val:" << std::endl;
    std::cout << val << std::endl;
    next_x = x - inv_jac_mat*val;
    if ((next_x - x).squaredNorm() < err) {
      x = next_x;
      return 0;
    }else { x = next_x; }
  }

  return 1;
}
