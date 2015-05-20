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
  }

  // simplest case 1: outer region
  {
    VectorField vf;
    Eigen::Vector3d u[2], c;
    c << -14.13, 43.2, -6.3;
    u[0] << -3.34, -8.3, 6.7;
    u[1] << 1.3, 5.8, -7.833134;
    std::shared_ptr<Function> ex_func(new LinearScalarField(u[0].data(), c.data()));
    std::shared_ptr<Function> fx_func(new LinearScalarField(u[1].data(), c.data()));

    double r[2] = {4.3, 6.9};
    std::shared_ptr<BlendFunc> br_func(new BlendFunc(r[0], r[1]));
    std::shared_ptr<RegionFunc> rx_func(new SphereRegionFunc(r[0], r[1], c.data()));
    vf.setExFunc(ex_func);
    vf.setFxFunc(fx_func);
    vf.setBrFunc(br_func);
    vf.setRxFunc(rx_func);

    // test val
    Eigen::Vector3d val_vf, x;
    x << -10.3, 40.2, -1.3;
    vf.val(x.data(), val_vf.data());
    if(val_vf.squaredNorm() > EPS) {
      suc = false;
      std::cerr << "[ERROR]" << __FILE__ << " line: " << __LINE__  << std::endl;
    }
  }

  // simplest case 2: blend region
  {
    VectorField vf;
    Eigen::Vector3d u[2], c;
    c << -14.13, 43.2, -6.3;
    u[0] << -3.34, -8.3, 6.7;
    u[1] << 1.3, 5.8, -7.833134;
    std::shared_ptr<Function> ex_func(new LinearScalarField(u[0].data(), c.data()));
    std::shared_ptr<Function> fx_func(new LinearScalarField(u[1].data(), c.data()));

    double r[2] = {4.3, 6.9};
    std::shared_ptr<BlendFunc> br_func(new BlendFunc(r[0], r[1]));
    std::shared_ptr<RegionFunc> rx_func(new SphereRegionFunc(r[0], r[1], c.data()));
    vf.setExFunc(ex_func);
    vf.setFxFunc(fx_func);
    vf.setBrFunc(br_func);
    vf.setRxFunc(rx_func);

    // test val
    Eigen::Vector3d val_vf, x, val_expected, jac_p, jac_q;
    x << -10.3, 42.1, -4.1;
    jac_p << -3.69317, -8.16737,  6.46809;
    jac_q << 1.90862,   5.6044, -7.45463;

    val_expected = jac_p.cross(jac_q);
    vf.val(x.data(), val_vf.data());
    if((val_vf-val_expected).squaredNorm() > EPS) {
      suc = false;
      std::cerr << "[ERROR]" << __FILE__ << " line: " << __LINE__  << std::endl;
      std::cerr <<  "val_expected:" << val_expected.transpose() << std::endl;
      std::cerr << "val_get:" << val_vf.transpose() << std::endl;
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
