#include <cassert>
#include <iostream>

#include <Eigen/Dense>

#define HAVE_CSTDDEF
#include <coin/IpTNLP.hpp>
#undef HAVE_CSTDDEF
#include <coin/IpIpoptApplication.hpp>

using namespace std;

using namespace Ipopt;

class TNLPIMP : public Ipopt::TNLP
{
public:
  TNLPIMP(Eigen::Matrix<Number,3,1> cx) : cx_(cx) {
    cn_=0;
  }

  // ax+by+cz>=d
  void addConstraint(Number a, Number b, Number c, Number d)
  {
    vec_a_.push_back(a);
    vec_b_.push_back(b);
    vec_c_.push_back(c);
    vec_d_.push_back(d);
    ++cn_;
  }

  bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
                    Index& nnz_h_lag, IndexStyleEnum& index_style)
  {
    n=3;
    m=cn_;
    nnz_jac_g = 3*cn_;
    nnz_h_lag=0;
    index_style=TNLP::C_STYLE;
    return true;
  }

  bool get_bounds_info(Index n, Number* x_l, Number* x_u,
                                 Index m, Number* g_l, Number* g_u)
  {
    assert(n==3);    assert(m==cn_); assert(cn_==vec_d_.size());
    copy(vec_d_.begin(), vec_d_.end(), g_l);
    return true;
  }

  bool get_starting_point(Index n, bool init_x, Number* x,
                          bool init_z, Number* z_L, Number* z_U,
                          Index m, bool init_lambda,
                          Number* lambda)
  {
    assert(n==3);
    assert(init_x==true);
    x[0]=cx_[0]; x[1]=cx_[1]; x[2]=cx_[2];
    assert(init_z==false); assert(init_lambda==false);
    return true;
  }

  bool eval_f(Index n, const Number* x, bool new_x,
                        Number& obj_value)
  {
    assert(n==3);
     obj_value = (Eigen::Map<const Eigen::Matrix<Number,3,1>>(x)- cx_).squaredNorm();
    return true;
  }


  bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
  {
    Eigen::Map<Eigen::Matrix<Number,3,1>> grad_f_e(grad_f);
    grad_f_e = (Eigen::Map<const Eigen::Matrix<Number,3,1>>(x)-cx_)*2;
    return true;
  }

   bool eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
  {
    assert(m==cn_);
    Eigen::Map<Eigen::Matrix<Number, Eigen::Dynamic,1>> col_a(vec_a_.data(), vec_a_.size());
    Eigen::Map<Eigen::Matrix<Number, Eigen::Dynamic,1>> col_b(vec_b_.data(), vec_b_.size());
    Eigen::Map<Eigen::Matrix<Number, Eigen::Dynamic,1>> col_c(vec_c_.data(), vec_c_.size());
    Eigen::Map<Eigen::Matrix<Number, Eigen::Dynamic,1>>ge(g, cn_);
    ge=col_a*x[0] + col_b*x[1] + col_c*x[2];
    return true;
  }

  bool eval_jac_g(Index n, const Number* x, bool new_x,
                  Index m, Index nele_jac, Index* iRow,
                  Index *jCol, Number* values)
  {
    if(values==NULL) {
      int nnz=0;
      for(int i=0; i<cn_; ++i) {
        iRow[nnz]=i; jCol[nnz]=0; ++nnz;
        iRow[nnz]=i; jCol[nnz]=1; ++nnz;
        iRow[nnz]=i; jCol[nnz]=2; ++nnz;
      }
    }
    else {
      *iRow=cn_;
      *jCol=3;
      Number *ptr=values;
      for(int i=0; i<cn_; ++i) {
        *ptr=vec_a_[i]; ++ptr;
        *ptr=vec_b_[i]; ++ptr;
        *ptr=vec_c_[i]; ++ptr;
      }
    }
    return true;
  }

  bool eval_h(Index n, const Number* x, bool new_x,
                        Number obj_factor, Index m, const Number* lambda,
                        bool new_lambda, Index nele_hess,
                        Index* iRow, Index* jCol, Number* values)
  {
    // hessian is all zero
    return true;
  }

  void finalize_solution(SolverReturn status,
                         Index n, const Number* x, const Number* z_L, const Number* z_U,
                         Index m, const Number* g, const Number* lambda,
                         Number obj_value,
                         const IpoptData* ip_data,
                         IpoptCalculatedQuantities* ip_cq)
  {
    if(status == SUCCESS) {
      std::cout << "success!\n x:"  << Eigen::Map<const Eigen::Matrix<Number,1,3>>(x) << std::endl;
      std::cout << "g:" << Eigen::Map<const Eigen::Matrix<Number,1,Eigen::Dynamic>>(g, cn_) << std::endl;
    } else {
      std::cerr << "failed!" << std::endl;
    }
  }
private:
  Ipopt::Index cn_; // number of constraint
  Eigen::Matrix<Number,3,1> cx_;
  vector<Number> vec_a_;
  vector<Number> vec_b_;
  vector<Number> vec_c_;
  vector<Number> vec_d_;
};

int main(int argc, char *argv[])
{
  Eigen::Matrix<Number,3,1> cx; cx<<1,1,1;
  SmartPtr<TNLPIMP> mynlp = new TNLPIMP(cx);

  mynlp->addConstraint(1,0,0,0);
  mynlp->addConstraint(0,1,0,0);
  mynlp->addConstraint(0,0,1,0);
  mynlp->addConstraint(1,1,1,1);

  SmartPtr<IpoptApplication> app = IpoptApplicationFactory();

  app->Options()->SetNumericValue("tol", 1e-3);
  app->Options()->SetStringValue("mu_strategy", "adaptive");
  app->Options()->SetStringValue("output_file", "ipopt.out");

  ApplicationReturnStatus status;
  status = app->Initialize();
  if (status != Solve_Succeeded) {
    printf("\n\n*** Error during initialization!\n");
    return (int) status;
  }

  // Ask Ipopt to solve the problem
  status = app->OptimizeTNLP(mynlp);

  if (status == Solve_Succeeded) {
    printf("\n\n*** The problem solved!\n");
  }
  else {
    printf("\n\n*** The problem FAILED!\n");
  }
  return 0;
}
