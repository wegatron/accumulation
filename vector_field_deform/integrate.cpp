#include "integrate.h"

using namespace zsw;

Eigen::Vector3d zsw::VectorFieldIntegrate::operator()(const double* pos) const
{
  const double h = 0.01; // the value is from Fernando

  if(vfs_.size() < 3) {
    Eigen::Vector3d vel;
    vfs_[vfs_.size()-1]->val(pos, vel.data());
    return vel*h;
  }

  Eigen::Vector3d ori_pos, tmp_pos;
  std::copy(pos, pos+3, oir_pos.data());
  Eigen::Vector3d k1;
  vfs_[vfs_.size()-3]->val(pos, k1.data());

  Eigen::Vector3d k2;
  tmp_pos = oir_pos + h*k1;
  vfs_[vfs_.size()-2]->val(tmp_pos.data(), k2.data());

  Eigen::Vector3d k3;
  tmp_pos = oir_pos +2*h*(-k1+2*k2);
  vfs_[vfs_.size()-1]->val(tmp_pos.data(), k3.data());
  return (k1+4*k2+k3)/6*h;
}

void zsw::VectorFieldIntegrate::pushVectorField(std::shared_ptr<VectorField> vf)
{
  if(vfs_.size()>=3) {
    vfs_->pop();
  }
  vfs_->push(vf);
}
