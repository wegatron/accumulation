#include "scalar_field.h"

#include <string.h>

zsw::LinearScalarField::LinearScalarField(const double *x, const double *u, const double *c)
{
  memcpy(x_, x, sizeof(double)*3);
  memcpy(u_, u, sizeof(double)*3);
  memcpy(c_, c, sizeof(double)*3);
}

double zsw::LinearScalarField::val()
{
  return u_[0]*(x_[0]-c_[0])+u_[1]*(x_[1]-c_[1])+u_[2]*(x_[2]-c_[2]);
}

void zsw::LinearScalarField::gra(double *g)
{
  memcpy(g, u_, sizeof(double)*3);
}

zsw::LinearScalarField::~LinearScalarField()
{
}
