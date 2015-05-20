#ifndef IMPLICIT_TOOLS_H
#define IMPLICIT_TOOLS_H

#include "vector_field.h"

namespace zsw{
  class ImplicitTool
  {
  public:
    virtual VectorField* genOrUpdateVectorField() = 0;
  };

  class SphereDeformTool final : public ImplicitTool
  {
  public:
    SphereDeformTool() {
      fill(center_, center_+6, 0.0);
      r_[0] = r_[1] = 0.0;
      pre = -1; now = 0;
    }
    VectorField* genOrUpdateVectorField();
    void updateCenter(const double *new_center);
  private:
    unsigned short pre, now;
    double center_[2][3];
    double r_[2];
  };

  class VfDeformer
  {
  public:
    void loadModel(const std::string& file_path);
    void saveModel(const std::string& fille_path);
    void pushVectorField(std::shared_ptr<VectorField> vf);
    void deform();
  private:
    std::shared_ptr<VectorFieldIntegrator> vf_integrator_;
  };
}

#endif /* IMPLICIT_TOOLS_H */
