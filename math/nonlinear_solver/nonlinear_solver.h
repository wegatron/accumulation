#ifndef NONLINEAR_SOLVER_H
#define NONLINEAR_SOLVER_H

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>

namespace zsw{
  class EquationsFunc{
  public:
    EquationsFunc(std::size_t dim) : dim_(dim) {}
    virtual ~EquationsFunc() {};
    virtual int val(const Eigen::VectorXd &x, Eigen::VectorXd &val_ret) const=0;
    virtual int jac(const Eigen::VectorXd &x, Eigen::MatrixXd &jac_ret) const=0;
    std::size_t dim() { return dim_; }
  private:
    size_t dim_;
  };
  typedef boost::shared_ptr<EquationsFunc> pEquationsFunc;

  /**
   * solve nonlinear equations
   * @return 0 get the result satisfied the requirement; 1 stoped for J is singular matrix;
   * 1 result can't satisfied the requirement after max_iter times iterations. If retcode is not these
   * means error occured.
   *
   * @val_ret is also the initial x_0 input into the solver.
   */
  int solve(const pEquationsFunc eq_func, const int max_iter, const double err,
            Eigen::VectorXd &x);
}

#endif /* NONLINEAR_SOLVER_H */
