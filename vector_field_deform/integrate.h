#ifndef INTEGRATE_H
#define INTEGRATE_H

#include <queue>

#include "vector_field.h"

namespace zsw {
  class VectorFieldIntegrate
  {
  public:
    VectorFieldIntegrate() {}
    Eigen::Vector3d operator()(const double* pos) const;
    void pushVectorField(std::shared_ptr<VectorField> vf);
  private:
      std::queue<std::shared_ptr<VectorField>> vfs_;
  };
}

#endif /* INTEGRATE_H */
