#include "vector_field.h"

#include <iostream>

zsw::VectorField::VectorField()
{
}

void zsw::VectorField::val(const double *x, double *val)
{
  std::cerr << "Function " << __FUNCTION__ << "in " << __FILE__ << __LINE__  << " haven't implement!!!" << std::endl;
}

void zsw::VectorField::setExFunc(std::shared_ptr<zsw::Function> ex_func)
{
  std::cerr << "Function " << __FUNCTION__ << "in " << __FILE__ << __LINE__  << " haven't implement!!!" << std::endl;
}


void zsw::VectorField::setFxFunc(std::shared_ptr<zsw::Function> fx_func)
{
  std::cerr << "Function " << __FUNCTION__ << "in " << __FILE__ << __LINE__  << " haven't implement!!!" << std::endl;
}

void zsw::VectorField::setBrFunc(std::shared_ptr<zsw::BlendFunc> br_func)
{
  std::cerr << "Function " << __FUNCTION__ << "in " << __FILE__ << __LINE__  << " haven't implement!!!" << std::endl;
}

void zsw::VectorField::setRxFunc(std::shared_ptr<zsw::RegionFunc> rx_func)
{
  std::cerr << "Function " << __FUNCTION__ << "in " << __FILE__ << __LINE__  << " haven't implement!!!" << std::endl;
}
