#include "scalar_field.h"

#include <iostream>
#include <string.h>

zsw::LinearScalarField::LinearScalarField(const double *u, const double *c)
{
  memcpy(u_, u, sizeof(double)*3);
  memcpy(c_, c, sizeof(double)*3);
}

double zsw::LinearScalarField::val(const double *x)
{
  return u_[0]*(x[0]-c_[0])+u_[1]*(x[1]-c_[1])+u_[2]*(x[2]-c_[2]);
}

void zsw::LinearScalarField::gra(const double *x, double *g)
{
  memcpy(g, u_, sizeof(double)*3);
}

zsw::LinearScalarField::~LinearScalarField(){}


zsw::BlendFunc::BlendFunc(const double ri, const double ro)
{
  ri_ = ri;
  ro_ = ro;
}

double zsw::BlendFunc::val(const double *r)
{
  std::cerr << "Function " << __FUNCTION__ << "in " << __FILE__ << __LINE__  << " haven't implement!!!" << std::endl;
  return 0.0;
}

void zsw::BlendFunc::gra(const double *r, double *g)
{
  std::cerr << "Function " << __FUNCTION__ << "in " << __FILE__ << __LINE__  << " haven't implement!!!" << std::endl;
}

zsw::BlendFunc::~BlendFunc() {}

zsw::SphereRegionFunc::SphereRegionFunc(const double ri, const double ro)
{
  ri_ = ri;
  ro_ = ro;
}

double zsw::SphereRegionFunc::val(const double *x)
{
  std::cerr << "Function " << __FUNCTION__ << "in " << __FILE__ << __LINE__  << " haven't implement!!!" << std::endl;
  return 0.0;
}

void zsw::SphereRegionFunc::gra(const double *x, double *g)
{
  std::cerr << "Function " << __FUNCTION__ << "in " << __FILE__ << __LINE__  << " haven't implement!!!" << std::endl;
}

zsw::RegionFunc::REGION_TYPE judgeRegion(const double *x)
{
  std::cerr << "Function " << __FUNCTION__ << "in " << __FILE__ << __LINE__  << " haven't implement!!!" << std::endl;
  return zsw::RegionFunc::INNER_REGION;
}
