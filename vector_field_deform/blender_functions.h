#ifndef BLENDER_FUNCTIONS_H
#define BLENDER_FUNCTIONS_H

#include "scalar_field.h"

namespace zsw{
  class Rigon
  {
  public:
    enum REGION_TYPE {
      INNER_REGION,
      BLENDER_REGION,
      OUTER_REGION
    };

    virtual REGION_TYPE judgeRegion(const double *x) = 0;
  };

  class SphereRegion : public Region, public ScalarField
  {
  public:
    virtual Region::REGION_TYPE judgeRegion(const double *x);
    virtual double val();
    virtual void gra(double *g);
  private:

  };
}


#endif /* BLENDER_FUNCTIONS_H */
