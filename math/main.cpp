#include "gauss_solver.h"

#include <iostream>
#include <ctime>

using namespace std;

void test_1()
{
  Eigen::MatrixXd A;
  Eigen::VectorXd B, x;
  A.resize(3,3); B.resize(3);
  A << -0.002, 2, 2,
    1, 0.78125, 0,
    3.996, 5.5625, 4;
  B << 0.4, 1.3816, 7.4178;
  if(zsw::solve(A, B, x) == 0) {
    cout << "result x:\n" << x.transpose() << endl;
  } else {
    cerr << "unable to solve the equation!" << endl;
  }
}

void test_2()
{
  Eigen::MatrixXd A;
  Eigen::VectorXd B, x;
  A.resize(3,3); B.resize(3);
  A << 1, 1, 1,
    0, 2, -3,
    0, 0, -7;
  B << 6, -5, -21;
  if(zsw::solve(A, B, x) == 0) {
    cout << "result x:\n" << x.transpose() << endl;
  } else {
    cerr << "unable to solve the equation!" << endl;
  }
}

void test_3()
{
  Eigen::MatrixXd A;
  Eigen::VectorXd B, x;
  A.resize(3,3); B.resize(3);
  A << 9,4,1,
    1, -2, -6,
    1, 6, 0;
  B << -17, 14, 4;
  if(zsw::solve(A, B, x) == 0) {
    cout << "result x:\n" << x.transpose() << endl;
  } else {
    cerr << "unable to solve the equation!" << endl;
  }
}

int test_random(int size)
{
  Eigen::MatrixXd A, cpA;
  Eigen::VectorXd B, cpB, x;
  A.resize(size, size); B.resize(size);
  srand(time(NULL));
  A.setRandom(); B.setRandom();
  cout << "A:\n" << A << endl;
  cout << "B: " << B.transpose() << endl;
  cpA = A; cpB=B;
  if(zsw::solve(cpA, cpB, x) == 0) {
    cout << "result x:\n" << x.transpose() << endl;
  } else {
    cerr << "unable to solve the equation!" << endl;
  }

  Eigen::VectorXd verify(size);
  verify = A*x;
  cout << "error vec's squared norm: " << (verify - B).squaredNorm() << endl;
  return 0;
}

int main(int argc, char *argv[])
{
  test_random(3);
  return 0;
}
