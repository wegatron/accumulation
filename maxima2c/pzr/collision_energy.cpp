#include "collision_energy.h"

#include "math.h"

double collisionEnergy(const double *x, const double *c, const double r)
{
  //user variables needed in the method
  double c1 = c[0];
  double c2 = c[1];
  double c3 = c[2];
  double x1 = x[0];
  double x2 = x[1];
  double x3 = x[2];

  //declare temporary variables
  double tt1;
  double tt2;
  double tt3;
  double tt4;
  double tt5;
  double tt6;
  double tt7;
  double tt8;
  double tt9;

  //calculate temporary variables
  tt1=(c1*c1);
  tt2=(c2*c2);
  tt3=(c3*c3);
  tt4=-2*c1*x1;
  tt5=(x1*x1);
  tt6=-2*c2*x2;
  tt7=(x2*x2);
  tt8=-2*c3*x3;
  tt9=(x3*x3);

  //outputs
  return -2*r*sqrt(tt9+tt8+tt7+tt6+tt5+tt4+tt3+tt2+tt1)+tt9+tt8+tt7+tt6+tt5+tt4+(r*r)+tt3+tt2+tt1;
}

void jacCollisionEnergy(const double *x, const double *c, const double r, double *out_jac)
{
  //user variables needed in the method
  double c1 = c[0];
  double c2 = c[1];
  double c3 = c[2];
  double x1 = x[0];
  double x2 = x[1];
  double x3 = x[2];

  //declare temporary variables
  double tt1;
  double tt2;
  double tt3;
  double tt4;
  double tt5;
  double tt6;
  double tt7;

  //calculate temporary variables
  tt1=-2*c1;
  tt2=2*x1;
  tt3=1/sqrt((x3*x3)-2*c3*x3+(x2*x2)-2*c2*x2+(x1*x1)-2*c1*x1+(c3*c3)+(c2*c2)+(c1*c1));
  tt4=-2*c2;
  tt5=2*x2;
  tt6=-2*c3;
  tt7=2*x3;

  //outputs
  out_jac[0]=-r*(tt2+tt1)*tt3+tt2+tt1;
  out_jac[1]=-r*(tt5+tt4)*tt3+tt5+tt4;
  out_jac[2]=-r*(tt7+tt6)*tt3+tt7+tt6;
}
