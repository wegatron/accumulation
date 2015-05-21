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
    std::shared_ptr<VectorFieldIntegrator> getVectorFieldIntegrator() { return vf_integrator_; }
    void pushVectorField(std::shared_ptr<VectorField> vf);
    void deform();
  private:
    std::shared_ptr<VectorFieldIntegrator> vf_integrator_;
  };

  class ImplicitTool
  {
  protected:
    virtual void updateVectorFieldAndDeform() = 0;
  };

  class SphereDeformTool final : public ImplicitTool
  {
  public:
    SphereDeformTool(const double *center, const double ri, const double ro) {
      center_[0] = center[0]; center_[1] = center[1]; center_[2] = center[2];
      r_[0] = ri; r_[1] = ro;
      n_  = 1000;
    }
    void setDeformer(std::shared_ptr<VfDeformer> deformer);
    void translateAndDeform(const double *trans_vec);
    void setNofSingleStep(size_t n) { n_ = n; }
  protected:
    void updateVectorFieldAndDeform();
  private:
    void calcU(const Eigen::Vector3d &u_dest, Eigen::Vector3d &u0, Eigen::Vector3d &u1);
    std::shared_ptr<VfDeformer> deformer_;
    double center_[3];
    double r_[2];
    size_t n_;

    // translate vector
    const double* trans_vec_;
  };

}

#endif /* IMPLICIT_TOOLS_H */
