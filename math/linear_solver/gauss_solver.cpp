#include "gauss_solver.h"

#include <iostream>

using namespace std;

/**
 * find the max value in col r, and swap it with row r.
 */
static int adjustRow(const int r, const int size, const double w,
                     Eigen::MatrixXd &A, Eigen::VectorXd &cpB)
{
  double max_val = fabs(A(r,r));
  int max_index = r;
  for (int i=r+1; i<size; ++i) {
    if (fabs(A(i,r)) > max_val) {
      max_val = fabs(A(i,r));
      max_index = i;
    }
  }
  if (max_val < w) {
    cerr << "det A == 0" << endl;
    return -1;
  }
  if(max_index != r) {
    Eigen::VectorXd tmp_row = A.row(r);
    A.row(r) = A.row(max_index);
    A.row(max_index) = tmp_row;

    double tmp = cpB(r);
    cpB(r)=cpB(max_index);
    cpB(max_index)=tmp;
  }
  return 0;
}

/**
 * gaussian elimination
 */
static int guassianElimation(const double w, Eigen::MatrixXd &cpA, Eigen::VectorXd &cpB)
{
  int size=cpA.cols();
  for (int i=0; i<size-1; ++i) {
    // find max fabs a_{kk} and swap it with row i
    if(adjustRow(i, size, w, cpA, cpB)) { return -1; }
    Eigen::Matrix<double, 1, -1> tmp(size-i);
    tmp = (-1/cpA(i,i)) * cpA.block(i,i, 1, size-i);
    for (int j=i+1; j<size; ++j) {
      cpB(j) -= cpB(i)*cpA(j,i)/cpA(i,i);
      cpA.block(j,i, 1,size-i) += cpA(j,i)*tmp;
    }
  }
  return 0;
}

int zsw::solve(const Eigen::MatrixXd &A, const Eigen::VectorXd &B, Eigen::VectorXd &x)
{
  assert(A.cols() == A.rows());
  assert(A.cols() == B.size());

  Eigen::MatrixXd cpA = A;
  Eigen::VectorXd cpB = B;
  int size = A.cols();
  const double w = 1e-10; // if the max coefficient < w, then means det(A)=0 solve faiil.
  if(guassianElimation(w, cpA, cpB)) { return -1; }
  // cout << "After guassian elimation, Matrix A:\n" << cpA << endl;
  // cout << "Vector B:\n" << cpB.transpose() << endl;
  // caculate the X
  x.resize(size);
  for (int i=size-1; i>=0; --i) {
    for (int j=i+1; j<size; ++j) {
      cpB(i) -= cpA(i,j) * x(j);
    }
    x(i) = cpB(i)/cpA(i,i);
  }
}
