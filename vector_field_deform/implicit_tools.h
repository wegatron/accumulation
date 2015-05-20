#ifndef IMPLICIT_TOOLS_H
#define IMPLICIT_TOOLS_H

#include "integrate.h"

namespace zsw{

  class VfDeformer final
  {
  public:
    VfDeformer() {}
    void loadModel(const std::string& file_path);
    void saveModel(const std::string& fille_path);
    void setVectorFieldIntegrator(std::shared_ptr<VectorFieldIntegrator> vf_integrator) { vf_integrator_ = vf_integrator; }
    void pushVectorField(std::shared_ptr<VectorField> vf);
    void deform();
  private:
    std::shared_ptr<VectorFieldIntegrator> vf_integrator_;
  };

  class ImplicitTool
  {
  public:
    virtual void updateVectorFieldAndDeform() = 0;
  };

  class SphereDeformTool final : public ImplicitTool
  {
  public:
    SphereDeformTool(const double *center) {
      center_[0][0] = center[0]; center_[0][1] = center[1]; center_[0][2] = center[2];
      center_[1][0] = center_[1][1] = center_[1][2] = 0.0;
      r_[0] = r_[1] = 0.0;
      now = 1;
    }
    void updateVectorFieldAndDeform();
    void updateCenter(const double *new_center);
    void setDeformer(std::shared_ptr<Deformer> deformer) { deformer_ = deformer; }
  private:
    std::shared_ptr<VfDeformer> deformer_;
    unsigned short now;
    double center_[2][3];
    double r_[2];
  };

}

#endif /* IMPLICIT_TOOLS_H */
