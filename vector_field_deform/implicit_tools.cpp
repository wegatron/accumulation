#include "implicit_tools.h"

#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include "VTKWriter.h"

using namespace std;
using namespace zsw;

void zsw::SphereDeformTool::calcU(const Eigen::Vector3d &u_dest, Eigen::Vector3d &u0, Eigen::Vector3d &u1)
{
  const double eps = 1e-6;
  size_t min_ind = 0;
  double len = u_dest.norm();
  if(len < eps) {    u0.setZero(); u1.setZero();    return;  }
  if(fabs(u_dest[1]) < fabs(u_dest[min_ind]) ) min_ind = 1;
  if(fabs(u_dest[2]) < fabs(u_dest[min_ind]) ) min_ind = 2;

  Eigen::Vector3d u = u_dest/len;
  u0[min_ind] = 0;
  switch(min_ind) {
  case 0:
    u0[1] = u[2];
    u0[2] = -u[1];
    break;
  case 1:
    u0[0] = u[2];
    u0[2] = -u[0];
    break;
  default: // 2
    u0[1] = u[0];
    u0[0] = -u[1];
    break;
  }
  u1 = u.cross(u0);
  if((u0.cross(u1)-u).squaredNorm() > eps) {
    u1 = -u1;
  }
  u1 *= sqrt(len);
  u0 *= sqrt(len);
  // std::cerr << "u_dest:" << u_dest.transpose() << std::endl;
  // std::cerr << "u0:" << u0.transpose() << std::endl;
  // std::cerr << "u1:" << u1.transpose() << std::endl;
  // std::cerr << "err u:" << (u0.cross(u1)-u_dest).squaredNorm() << std::endl;
  assert((u0.cross(u1)-u_dest).squaredNorm() < eps);
}

void zsw::SphereDeformTool::setDeformer(std::shared_ptr<VfDeformer> deformer)
{
  assert(deformer != nullptr);
  deformer_ = deformer;
  deformer_->getVectorFieldIntegrator()->setStep(1.0/time_slice_);
}

void zsw::SphereDeformTool::updateVectorFieldAndDeform()
{
  assert(deformer_ != nullptr);
  Eigen::Vector3d u[3];
  u[2] << trans_vec_[0], trans_vec_[1], trans_vec_[2];
  calcU(u[2], u[0], u[1]);
  Eigen::Vector3d tmp_center;
  tmp_center << center_[0], center_[1], center_[2];
  u[2] /= time_slice_;
  for(size_t i=0; i<time_slice_; ++i) {
    std::cout << "step " << i << std::endl;
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
    deformer_->pushVectorFieldAndDeform(vf);
    tmp_center += u[2];

    // sequence out put
    #if 1
    static size_t counter = -1;
    writeVtk("/home/wegatron/tmp/se_"+std::to_string(++counter)+".vtk", deformer_->getVerts(), deformer_->getTris());
    #endif
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
  ifstream ifs(file_path);
  if(!ifs) {
    cerr << "can not openfile: " << file_path << endl;
    exit(1);
  }

  string str;
  double tmp_double[3];
  size_t tmp_int;

  vector<double> verts_v;
  vector<size_t> tris_v;

  while(!ifs.eof()) {
    ifs >> str;
    if(str == "v") {
      ifs >> tmp_double[0] >> tmp_double[1] >> tmp_double[2];
      verts_v.push_back(tmp_double[0]);
      verts_v.push_back(tmp_double[1]);
      verts_v.push_back(tmp_double[2]);
    }

    if(str=="f") {
      for (size_t i=0; i<3; ++i) {
        ifs >> str;
        stringstream ss(str);
        ss >> tmp_int;
        tris_v.push_back(tmp_int-1);
      }
    }
  }

  ifs.close();
  assert(verts_v.size()%3==0); assert(tris_v.size()%3==0);
  verts_.resize(3,verts_v.size()/3);
  tris_.resize(3,tris_v.size()/3);

  std::copy(verts_v.begin(), verts_v.end(), verts_.data());
  std::copy(tris_v.begin(), tris_v.end(), tris_.data());
}

void zsw::VfDeformer::saveModel(const std::string& file_path)
{
  ofstream ofs(file_path);
  if(!ofs) {
    cerr << "can open file: " << file_path << " for write!" << endl;
    exit(1);
  }
  for (int i=0; i<verts_.cols(); ++i) {
    ofs << "v " << verts_(0,i) << " " << verts_(1,i) << " " << verts_(2,i) << endl;
  }
  for (int i=0; i<tris_.cols(); ++i) {
    ofs << "f " << tris_(0,i)+1 << " " << tris_(1,i)+1 << " " << tris_(2,i)+1 << endl;
  }
  ofs.close();
}

void zsw::VfDeformer::pushVectorFieldAndDeform(std::shared_ptr<VectorField> vf)
{
  assert(vf_integrator_ != nullptr);
  vf_integrator_->pushVectorField(vf);
  assert(verts_.cols()!=0);
  for(size_t i=0; i<verts_.cols(); ++i) {
    Eigen::Vector3d pos = verts_.block<3,1>(0,i);
    verts_.block<3,1>(0,i) += (*vf_integrator_)(pos.data());
  }
}

void zsw::writeVtk(const std::string& file_path, Eigen::Matrix<double, 3, Eigen::Dynamic> &verts,
                const Eigen::Matrix<size_t, 3, Eigen::Dynamic>& tris)
{
  vector<Eigen::Vector3d> vverts;
  vector<Eigen::Vector3i> vtris;
  UTILITY::VTKWriter writer(true); // binary = true

  for(size_t i=0; i<verts.cols(); ++i) {
    Eigen::Vector3d tmp_v = verts.block<3,1>(0,i);
    vverts.push_back(tmp_v);
  }

  for(size_t i=0; i<tris.cols(); ++i) {
    Eigen::Matrix<size_t, 3, 1> tmp_v0 = tris.block<3,1>(0,i);
    Eigen::Vector3i tmp_v;
    tmp_v << tmp_v0[0], tmp_v0[1], tmp_v0[2];
    vtris.push_back(tmp_v);
  }

  writer.addPoints(vverts);
  writer.addTriangles(vtris);
  bool suc = writer.write(file_path);
  assert(suc);
}
