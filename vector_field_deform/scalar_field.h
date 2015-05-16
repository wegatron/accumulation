#ifndef SCALAR_FIELD_H
#define SCALAR_FIELD_H

namespace zsw {

  class Funtion
  {
  public:
    virtual double val() = 0;
    virtual void gra(double *g) = 0;
  };

  /**
   * function e(x) and f(x)
   */
  class LinearScalarField final : public Function
  {
  public:
    LinearScalarField(const double *x, const double *u, const double *c);
    virtual double val();
    virtual void gra(double *g);
    ~LinearScalarField();
  private:
    double x_[3];
    double u_[3];
    double c_[3];
  };

  /**
   * function b(r)
   */
  class BlendFunc final : public Function
  {
  public:
    BlendFunc(const double ri, const double ro);
    virtual double val();
    virtual void gra(double *g);
    ~BlendFunc();
  private:
    double ri_;
    double ro_;
  };

  /**
   * function r(x)
   */
  class RegionFunc
  {
  public:
    enum REGION_TYPE {
      INNER_REGION,
      BLENDER_REGION,
      OUTER_REGION
    };
    virtual double val() = 0;
    virtual void gra(double *g) = 0;
    virtual Region::REGION_TYPE judgeRegion(const double *x) = 0;
  };
}

#endif /* SCALAR_FIELD_H */
