#include <iostream>
#include <vector>
#include <random>

#include <Eigen/Dense>
#include <chrono>

#include "scalar_field.h"

using namespace std;
using namespace zsw;

void test0()
{
  double x[3], u[3], c[3], g[3];

  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator (seed);

  std::uniform_real_distribution<double> distribution(-1000.0,1000.0);
  Eigen::Vector3d vx, vu, vc;

  for(size_t i=0; i<3; ++i) {
    vx[i] = x[i] = distribution(generator);
    vu[i] = u[i] = distribution(generator);
    vc[i] = c[i] = distribution(generator);
  }

  std::cout << x[0] << " " << x[1] << " " << x[2] << std::endl;
  std::cout << u[0] << " " << u[1] << " " << u[2] << std::endl;
  std::cout << c[0] << " " << c[1] << " " << c[2] << std::endl;
  LinearScalarField lsf(x, u, c);
  std::cout << lsf.val() <<  " " << vu.dot(vx-vc) << std::endl;
  if(fabs(lsf.val()-vu.dot(vx-vc))>1e-6) { std::cout << "error!" << std::endl; }
  lsf.gra(g);
  std::cout << g[0] << " " << g[1] << " " << g[2] << std::endl;
}

int main(int argc, char *argv[])
{
  test0();
  return 0;
}
