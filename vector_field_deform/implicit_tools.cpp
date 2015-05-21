#include "implicit_tools.h"

#include <iostream>

#include <Eigen/Dense>

using namespace zsw;

void zsw::SphereDeformTool::calcU(const Eigen::Vector3d &u_dest, Eigen::Vector3d &u0, Eigen::Vector3d &u1)
{
  size_t min_ind = 0;
  if(fabs(u_dest[1]) < fabs(u_dest[min_ind]) ) min_ind = 1;
  if(fabs(u_dest[2]) < fabs(u_dest[min_ind]) ) min_ind = 2;

  u0[min_ind] = 0;
  switch(min_ind) {
  case 0:
    u0[1] = u_dest[2];
    u0[2] = -u_dest[1];
    break;
  case 1:
    u0[0] = u_dest[2];
    u0[2] = -u_dest[0];
    break;
  default: // 2
    u0[1] = u_dest[0];
    u0[0] = -u_dest[1];
    break;
  }
  u1 = u_dest.cross(u0);
}

void zsw::SphereDeformTool::setDeformer(std::shared_ptr<VfDeformer> deformer)
{
  assert(deformer != nullptr);
  deformer_ = deformer;
  deformer_->getVectorFieldIntegrator()->setStep(1.0/n_);
}

void zsw::SphereDeformTool::updateVectorFieldAndDeform()
{
  assert(deformer_ != nullptr);
  Eigen::Vector3d u[3];
  u[2] << trans_vec_[0], trans_vec_[1], trans_vec_[2];
  calcU(u[2], u[0], u[1]);
  // every single step's u
  u[0] /= n_; u[1] /= n_;
  Eigen::Vector3d tmp_center;
  tmp_center << center_[0], center_[1], center_[2];
  for(size_t i=0; i<n_; ++i) {
    // generate ex, fx, rx, br set into vf
    std::shared_ptr<VectorField> vf(new VectorField());
    std::shared_ptr<Function> ex_func(new LinearScalarField(u[0].data(), tmp_center.data()));
    std::shared_ptr<Function> fx_func(new LinearScalarField(u[1].data(), tmp_center.data()));
    std::shared_ptr<RegionFunc> rx_func(new SphereRegionFunc(r_[0], r_[1], tmp_center.data()));
    std::shared_ptr<BlendFunc> br_func(new BlendFunc(r_[0], r_[1]));

    vf->setExFunc(ex_func);
    vf->setFxFunc(fx_func);
    vf->setBrFunc(br_func);
    vf->setRxFunc(rx_func);

    deformer_->pushVectorField(vf);
    deformer_->deform();
  }
}

void zsw::SphereDeformTool::translateAndDeform(const double *trans_vec)
{
  trans_vec_ = trans_vec;
  updateVectorFieldAndDeform();
  center_[0] += trans_vec[0];
  center_[1] += trans_vec[1];
  center_[2] += trans_vec[2];
}

void zsw::VfDeformer::loadModel(const std::string& file_path)
{
  std::cerr << "Function " << __FUNCTION__ << "in " << __FILE__ << __LINE__  << " haven't implement!!!" << std::endl;
}

void zsw::VfDeformer::saveModel(const std::string& fille_path)
{
  std::cerr << "Function " << __FUNCTION__ << "in " << __FILE__ << __LINE__  << " haven't implement!!!" << std::endl;
}

void zsw::VfDeformer::pushVectorField(std::shared_ptr<VectorField> vf)
{
  std::cerr << "Function " << __FUNCTION__ << "in " << __FILE__ << __LINE__  << " haven't implement!!!" << std::endl;
}

void zsw::VfDeformer::deform()
{
  std::cerr << "Function " << __FUNCTION__ << "in " << __FILE__ << __LINE__  << " haven't implement!!!" << std::endl;
}
