#include "jac_hes_err.h"

template<typename val_type>
val_type maxFabsArray(const val_type *data, const int size) {
  val_type max_val = fabs(data[0]);
  for (int i=1; i<size; ++i) {
    double v = fabs(data[i]);
    max_val = (v>max_val) ? v : max_val;
  }
  return max_val;
}

template<typename val_type>
val_type maxFabsSparseMat(const Eigen::SparseMatrix<val_type> &mat)
{
  val_type max_val = 0;
  for (int i = 0; i < mat.outerSize(); ++i) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(mat,i); it; ++it) {
      max_val = (it.value()>max_val) ? it.value() : max_val;
    }
  }
  return max_val;
}

template<typename EnergyType>
double graErr(EnergyType &energy, Eigen::VectorXd &x)
{
  Eigen::VectorXd g;
  energy.force(x,g);
  std::cerr << "max fabs(g):" << maxFabsArray(&g[0],g.size()) << endl;
  const double eps = 1e-6;
  for(int i=0; i<g.size(); ++i) {
    double save = x[i];
    double v[2] = {0,0};
    x[i] = save + eps;
    v[0] = energy.val(x);
    x[i] = save - eps;
    v[1] = energy.val(x);
    g[i] -= (v[0]-v[1])/(2*eps);
    x[i] = save;
  }
  return maxFabsArray(&g[0], g.size());
}

template<typename EnergyType>
double hesErr(EnergyType &energy, Eigen::VectorXd &x)
{
  const Eigen::SparseMatrix<double> &K = energy.hes(x);
  Eigen::MatrixXd dense_k(K);

  std::cerr << "max fabs(K):" << maxFabsSparseMat(K) << endl;
  Eigen::VectorXd g[2];
  g[0].resize(x.size()); g[1].resize(x.size());
  const double eps = 1e-6;
  for (int c=0; c<x.size(); ++c) {
    double save = x[c];
    x[c] = save + eps;
    energy.jac(x,g[0]);
    x[c] = save - eps;
    energy.jac(x,g[1]);
    dense_k.col(c) -= (g[0]-g[1])/(2*eps);
    x[c] = save;
  }
  return maxFabsArray(dense_k.data(), dense_k.size());
}
