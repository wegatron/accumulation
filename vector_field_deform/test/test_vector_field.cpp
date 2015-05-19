#include <iostream>

#include <Eigen/Dense>
#include "../vector_field.h"

using namespace std;
using namespace zsw;

#define EPS 1e-5

void testValSpecific()
{
  bool suc = true;

  // simplest case 0 : inner region
  {
    VectorField vf;
    Eigen::Vector3d u[2], c;
    u[0] << 1,0,0; u[1] << 0,1,0;  c << 0,0,0;
    std::shared_ptr<Function> ex_func(new LinearScalarField(u[0].data(), c.data()));
    std::shared_ptr<Function> fx_func(new LinearScalarField(u[1].data(), c.data()));

    double r[2] = {1.0, 2.0};
    std::shared_ptr<BlendFunc> br_func(new BlendFunc(r[0], r[1]));
    std::shared_ptr<RegionFunc> rx_func(new SphereRegionFunc(r[0], r[1], c.data()));
    vf.setExFunc(ex_func);
    vf.setFxFunc(fx_func);
    vf.setBrFunc(br_func);
    vf.setRxFunc(rx_func);

    // test val
    Eigen::Vector3d val_vf, x, val_expected;
    x<< 0.7, 0.7, 0; val_expected << 0, 0, 1;

    vf.val(x.data(), val_vf.data());
    if((val_vf - val_expected).squaredNorm() > EPS) {
      suc = false;
      std::cerr << "[ERROR]" << __FILE__ << " line: " <<  __LINE__  << std::endl;
      std::cerr << val_vf.transpose() << std::endl;
    }

    // simplest case 1: outer region
    {
      std::cerr << "Function " << __FUNCTION__ << "in " << __FILE__ << __LINE__  << " haven't implement!!!" << std::endl;
    }
  }
  if(suc) { std::cout << "[INFO]" << __FUNCTION__ << "passed!" << std::endl; }
  suc = false;
}

  void testValRandom(size_t times)
  {
    bool suc = true;
    //body
    //if(suc) { std::cout << "[INFO]" << __FUNCTION__ << "passed!" << std::endl; }
  }

  int main(int argc, char *argv[])
  {
    testValSpecific();
    return 0;
  }
