#ifndef COLLISION_ENERGY_H
#define COLLISION_ENERGY_H

double collisionEnergy(const double *x, const double *c, const double r);

void jacCollisionEnergy(const double *x, const double *c, const double r, double *out_jac);

inline bool inball(const double *x, const double *c, const double r)
{
  double d0 = x[0] - c[0];
  double d1 = x[1] - c[1];
  double d2 = x[2] - c[2];

  return (d0*d0+d1*d1+d2*d2) < r*r;
}

#endif /* COLLISION_ENERGY_H */
