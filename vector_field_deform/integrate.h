#ifndef INTEGRATE_H
#define INTEGRATE_H

#include <vector>
#include <Eigen/Dense>

#include "vector_field.h"

namespace zsw {
  class VectorFieldIntegrate
  {
  public:
    VectorFieldIntegrate() { h_=0.01; // the value is from Fernando }
    Eigen::Vector3d operator()(const double* pos) const;
    void pushVectorField(std::shared_ptr<VectorField> vf);
    void setStep(const double h) { h_ = h; }
  private:
      std::vector<std::shared_ptr<VectorField>> vfs_;
      double h_;
  };
}

#endif /* INTEGRATE_H */
