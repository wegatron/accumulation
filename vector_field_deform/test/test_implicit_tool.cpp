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
  vf_deformer->loadModel("/home/wegatron/input.obj");
  // creatr implicit_tool
  double center[3] = {0,0,0};
  SphereDeformTool spdf_tool(center);
  spdf_tool.setDeformer(vf_deformer);
  double cs[3][3] = {{0.05,0,0}, {0.1,0,0}, {0.2,0,0}};
  // doing deform
  {
    spdf_tool.updateCenter(cs[0]);
    spdf_tool.updateVectorFieldAndDeform();

    spdf_tool.updateCenter(cs[1]);
    spdf_tool.updateVectorFieldAndDeform();

    spdf_tool.updateCenter(cs[2]);
    spdf_tool.updateVectorFieldAndDeform();
  }
  vf_deformer->saveModel("/home/wegatron/output.obj");
  return 0;
}
