#ifndef IMPLICIT_TOOLS_H
#define IMPLICIT_TOOLS_H

#include "integrate.h"

namespace zsw{

  void writeVtk(const std::string& file_path, Eigen::Matrix<double, 3, Eigen::Dynamic> &verts,
                const Eigen::Matrix<size_t, 3, Eigen::Dynamic>& tris);

  class VfDeformer final
  {
  public:
    VfDeformer() {}
    void loadModel(const std::string& file_path);
    void saveModel(const std::string& file_path);
    void setVectorFieldIntegrator(std::shared_ptr<VectorFieldIntegrator> vf_integrator) { vf_integrator_ = vf_integrator; }
    std::shared_ptr<VectorFieldIntegrator> getVectorFieldIntegrator() { return vf_integrator_; }
    void pushVectorFieldAndDeform(std::shared_ptr<VectorField> vf);

    Eigen::Matrix<double, 3, Eigen::Dynamic>& getVerts() { return verts_; }
    Eigen::Matrix<size_t, 3, Eigen::Dynamic>& getTris() { return tris_; }
  private:
    std::shared_ptr<VectorFieldIntegrator> vf_integrator_;
    Eigen::Matrix<double, 3, Eigen::Dynamic> verts_; // default column major
    Eigen::Matrix<size_t, 3, Eigen::Dynamic> tris_; // index start from 0
  };

  class ImplicitTool
  {
  public:
    void setDeformer(std::shared_ptr<VfDeformer> deformer);
    void setTimeSlice(size_t time_slice);
  protected:
    virtual void updateVectorFieldAndDeform() = 0;
    size_t time_slice_;
  };

  class SphereDeformTool final : public ImplicitTool
  {
  public:
    SphereDeformTool(const double *center, const double ri, const double ro) {
      center_[0] = center[0]; center_[1] = center[1]; center_[2] = center[2];
      r_[0] = ri; r_[1] = ro;
      time_slice_  = 100;
    }
    void setDeformer(std::shared_ptr<VfDeformer> deformer);
    void translateAndDeform(const double *trans_vec);
    void setTimeSlice(size_t time_slice) {
      time_slice_ = time_slice;
      if(deformer_ != nullptr)
        deformer_->getVectorFieldIntegrator()->setStep(1.0/time_slice_);
    }
  protected:
    void updateVectorFieldAndDeform();
  private:
    void calcU(const Eigen::Vector3d &u_dest, Eigen::Vector3d &u0, Eigen::Vector3d &u1);
    std::shared_ptr<VfDeformer> deformer_;
    double center_[3];
    double r_[2];
    size_t time_slice_;

    // translate vector
    const double* trans_vec_;
  };

  class BendDeformTool final : public ImplicitTool
  {
    void updateVectorFieldAndDeform();
  };
}

#endif /* IMPLICIT_TOOLS_H */
