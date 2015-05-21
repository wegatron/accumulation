#include <iostream>

#include "../implicit_tools.h"

using namespace std;
using namespace zsw;

int main(int argc, char *argv[])
{
  // create integrator
  std::shared_ptr<VectorFieldIntegrator> vf_integrator(new AdVectorIntegrator());
  // create deformer
  std::shared_ptr<VfDeformer> vf_deformer(new VfDeformer());
  vf_deformer->setVectorFieldIntegrator(vf_integrator);
  vf_deformer->loadModel("/home/wegatron/tmp/input.obj");
  // creatr implicit_tool
  double center[3] = {0,0,0};
  SphereDeformTool spdf_tool(center, 0.1, 0.15);
  spdf_tool.setDeformer(vf_deformer);
  double trans_vec[3][3] = {{0,0,0.1}, {0,0,0.1}, {0,0,0.1}};
  // doing deform
  {
    spdf_tool.translateAndDeform(trans_vec[0]);
    spdf_tool.translateAndDeform(trans_vec[1]);
    spdf_tool.translateAndDeform(trans_vec[2]);
  }
  vf_deformer->saveModel("/home/wegatron/tmp/output.obj");
  return 0;
}
