#include "implicit_tools.h"

#include <iostream>

using namespace zsw;

void zsw::SphereDeformTool::updateVectorFieldAndDeform()
{
  std::cerr << "Function " << __FUNCTION__ << "in " << __FILE__ << __LINE__  << " haven't implement!!!" << std::endl;
}

void zsw::SphereDeformTool::updateCenter(const double *new_center)
{
  std::cerr << "Function " << __FUNCTION__ << "in " << __FILE__ << __LINE__  << " haven't implement!!!" << std::endl;
  // center_[now][0] = new_center[0];
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
