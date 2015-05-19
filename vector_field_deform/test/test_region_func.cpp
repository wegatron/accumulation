#include <iostream>
#include <chrono>

#include <Eigen/Dense>

#include "../scalar_field.h"
#include "jac_hes_err.h"

using namespace std;

#define EPS 1e-6

void testValSpecific()
{
  bool suc = true;

  // group 1
  {
    double c[3] = {0,0,0};
    SphereRegionFunc spf(10, 35, c);
    double x[3] = {3.2,-1.4, 2.7};
    if( fabs(spf.val(x) -  4.414748010928823)> EPS) {
      std::cerr << "[ERROR]" << __FILE__ << __LINE__ << std::endl;
      std::cerr << "val:" << spf.val(x) << std::endl;
      suc = false;
    }
    if(spf.judgeRegion(x) != zsw::RegionFunc::INNER_REGION) {
      std::cerr << "[ERROR]" << __FILE__ << __LINE__ << std::endl;
    }
  }

  // group 2
  {
    double c[3] = {87.2,45.3,92.4};
    SphereRegionFunc spf(100, 120, c);
    double x[3] = {3.2,-1.4, 2.7};
    if( fabs(spf.val(x) -  131.4647481266365)> EPS) {
      std::cerr << "[ERROR]" << __FILE__ << __LINE__ << std::endl;
      std::cerr << "val:" << spf.val(x) << std::endl;
      suc = false;
    }
    if(spf.judgeRegion(x) != zsw::RegionFunc::OUTER_REGION) {
      std::cerr << "[ERROR]" << __FILE__ << __LINE__ << std::endl;
    }
  }

  //boundary ri
  {
    double c[3] = {67.182, 87.12, -123.12};
    SphereRegionFunc spf(92.85, 126, c);
    double x[3] = {1.2, 30.4, -90.7};
    if(fabs(spf(x) - 92.85385896127312)>EPS) {
      std::cerr << "[ERROR]" << __FILE__ << __LINE__ << std::endl;
      std::cerr << "val:" << spf.val(x) << std::endl;
      suc = false;
    }
    if(spf.judgeRegion(x) != zsw::RegionFunc::INNER_REGION) {
      std::cerr << "[ERROR]" << __FILE__ << __LINE__ << std::endl;
    }
  }

  //boundary ri
  {
    double c[3] = {67.182, 87.12, -123.12};
    SphereRegionFunc spf(92.85, 126, c);
    double x[3] = {0, 30.4, -90.7};
    if(fabs(spf(x) - 93.71038322405901)>EPS) {
      std::cerr << "[ERROR]" << __FILE__ << __LINE__ << std::endl;
      std::cerr << "val:" << spf.val(x) << std::endl;
      suc = false;
    }
    if(spf.judgeRegion(x) != zsw::RegionFunc::BLENDER_REGION) {
      std::cerr << "[ERROR]" << __FILE__ << __LINE__ << std::endl;
    }
  }

  //boundary ro
  {
    double c[3] = {62.19, -26.2, 57.0};
    SphereRegionFunc spf(92.85, 165, c);
    double x[3] = {11.2, 25.4, -90.7};
    if(fabs(spf(x) - 164.553426278519)>EPS) {
      std::cerr << "[ERROR]" << __FILE__ << __LINE__ << std::endl;
      std::cerr << "val:" << spf.val(x) << std::endl;
      suc = false;
    }
    if(spf.judgeRegion(x) != zsw::RegionFunc::BLENDER_REGION) {
      std::cerr << "[ERROR]" << __FILE__ << __LINE__ << std::endl;
    }
  }

  //boundary ro
  {
    double c[3] = {62.19, -26.2, 57.0};
    SphereRegionFunc spf(92.85, 165, c);
    double x[3] = {11.2, 25.4, -91.7};
    if(fabs(spf(x) - 165.4515944317249)>EPS) {
      std::cerr << "[ERROR]" << __FILE__ << __LINE__ << std::endl;
      std::cerr << "val:" << spf.val(x) << std::endl;
      suc = false;
    }
    if(spf.judgeRegion(x) != zsw::RegionFunc::OUTER_REGION) {
      std::cerr << "[ERROR]" << __FILE__ << __LINE__ << std::endl;
    }
  }

  if(suc) { std::cout << "[INFO]" << __FUNCTION__ << "passed!" << std::endl; }
}

void testValRandom(const size_t time)
{
}

void testGraErr(const size_t time)
{
}

int main(int argc, char *argv[])
{
  testValSpecific();
  testValRandom(100);
  testGraErr(100);
  return 0;
}
