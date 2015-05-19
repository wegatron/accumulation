#include "scalar_field.h"

#include <iostream>
#include <stdexcept>
#include <string>
#include <string.h>
#include <assert.h>
#include <cmath>

zsw::LinearScalarField::LinearScalarField(const double *u, const double *c)
{
  memcpy(u_, u, sizeof(double)*3);
  memcpy(c_, c, sizeof(double)*3);
}

double zsw::LinearScalarField::val(const double *x)
{
  return u_[0]*(x[0]-c_[0])+u_[1]*(x[1]-c_[1])+u_[2]*(x[2]-c_[2]);
}

void zsw::LinearScalarField::jac(const double *x, double *g)
{
  memcpy(g, u_, sizeof(double)*3);
}

zsw::LinearScalarField::~LinearScalarField(){}

zsw::BlendFunc::BlendFunc(const double ri, const double ro)
{
  ri_ = ri;
  ro_ = ro;
  assert(ro_>ri_);
}

double zsw::BlendFunc::val(const double *r)
{
  if(*r<ri_ || *r>ro_) {
    std::cerr << "[ERROR] r in blend func should in (ri,ro):" << ri_ << " " << ro_ << std::endl;
    throw std::logic_error("[ERROR] r in blend func should in (ri,ro):"+std::to_string(ri_)+" "+std::to_string(ro_));
  }
  double v= (*r-ri_)/(ro_-ri_);
  return 4*v*v*v*(1-v) + v*v*v*v;
}

void zsw::BlendFunc::jac(const double *r, double *g)
{
  g[0] = 12*(*r-ri_)*(*r-ri_)*(ro_-*r)/((ro_-ri_)*(ro_-ri_)*(ro_-ri_)*(ro_-ri_));
}

zsw::BlendFunc::~BlendFunc() {}

zsw::SphereRegionFunc::SphereRegionFunc(const double ri, const double ro, const double *center)
{
  ri_ = ri;
  ro_ = ro;
  memcpy(center_, center, sizeof(double)*3);
}

double zsw::SphereRegionFunc::val(const double *x)
{
  return  sqrt((x[0]-center_[0])*(x[0]-center_[0])+(x[1]-center_[1])*(x[1]-center_[1])+(x[2]-center_[2])*(x[2]-center_[2]));
}

void zsw::SphereRegionFunc::jac(const double *x, double *g)
{
  g[0] = 2*(x[0]-center_[0]);   g[1] = 2*(x[1]-center_[1]);   g[2] = 2*(x[2]-center_[2]);
}

zsw::RegionFunc::REGION_TYPE zsw::SphereRegionFunc::judgeRegion(const double *x)
{
  double v = (x[0]-center_[0])*(x[0]-center_[0])+(x[1]-center_[1])*(x[1]-center_[1])+(x[2]-center_[2])*(x[2]-center_[2]);
  if(v > ro_*ro_)   {
    return zsw::RegionFunc::OUTER_REGION;
  }
  if(v < ri_*ri_) {
    return zsw::RegionFunc::INNER_REGION;
  }
  return zsw::RegionFunc::BLENDER_REGION;
}
