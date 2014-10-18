#include <iostream>
#include <cmath>

#include "nonlinear_solver.h"

using namespace std;

namespace zsw{

  class EquationsFunc_0 : public EquationsFunc
  {
  public:
    EquationsFunc_0(std::size_t dim) : EquationsFunc(dim) {}
    ~EquationsFunc_0() {}
    int val(const Eigen::VectorXd &x, Eigen::VectorXd &val_ret) const
    {
      val_ret.resize(2);
      val_ret(0) = x(0)*x(0)+x(1)*x(1)-4;
      val_ret(1) = x(0)*x(0)-x(1)*x(1)-1;
      return 0;
    }
    int jac(const Eigen::VectorXd &x, Eigen::MatrixXd &jac_ret) const
    {
      jac_ret.resize(2,2);
      jac_ret(0,0)=2*x(0); jac_ret(0,1)=2*x(1);
      jac_ret(1,0)=jac_ret(0,0); jac_ret(1,1)=-jac_ret(0,1);
      return 0;
    }
  };
}

int main(int argc, char *argv[])
{
  zsw::pEquationsFunc eq_func(new zsw::EquationsFunc_0(2));
  Eigen::VectorXd x(2);
  x << 1.6, 1.2;
  int ret_code = zsw::solve(eq_func, 10, 1e-8, x);
  cout << "ret_code:" << ret_code << " x:" << x.transpose() << endl;
  Eigen::VectorXd val_ret;
  eq_func->val(x, val_ret);
  cout << "val_ret: " << val_ret.transpose() << endl;
  return 0;
}
