#include <iostream>

#include "../implicit_tools.h"

using namespace std;
using namespace zsw;

void testPlane()
{
  // create integrator
  std::shared_ptr<VectorFieldIntegrator> vf_integrator(new AdVectorIntegrator());
  // create deformer
  std::shared_ptr<VfDeformer> vf_deformer(new VfDeformer());
  vf_deformer->setVectorFieldIntegrator(vf_integrator);
  vf_deformer->loadModel("/home/wegatron/tmp/plane_input.obj");
  // creatr implicit_tool
  double center[3] = {0,0,0};
  SphereDeformTool spdf_tool(center, 0.01, 0.8);
  spdf_tool.setDeformer(vf_deformer);
  spdf_tool.setTimeSlice(10);
  double trans_vec[3][3] = {{0,0.1,0}, {0,0.1,0}, {0,0.1,0}};
  // doing deform
  {
    spdf_tool.translateAndDeform(trans_vec[0]);
    spdf_tool.translateAndDeform(trans_vec[1]);
    spdf_tool.translateAndDeform(trans_vec[2]);
  }
  vf_deformer->saveModel("/home/wegatron/tmp/plane_final.obj");
}

void testSphere()
{
  // create integrator
  std::shared_ptr<VectorFieldIntegrator> vf_integrator(new AdVectorIntegrator());
  // create deformer
  std::shared_ptr<VfDeformer> vf_deformer(new VfDeformer());
  vf_deformer->setVectorFieldIntegrator(vf_integrator);
  vf_deformer->loadModel("/home/wegatron/tmp/sphere_input.obj");
  // creatr implicit_tool
  double center[3] = {0,1,0};
  SphereDeformTool spdf_tool(center, 0.01, 1.0);
  spdf_tool.setDeformer(vf_deformer);
  spdf_tool.setTimeSlice(10);
  double trans_vec[2][3] = {{0,0.1,0}, {0.1, 0,0}};
  size_t time = 20;
  // doing deform
  do {
    spdf_tool.translateAndDeform(trans_vec[0]);
    // spdf_tool.translateAndDeform(trans_vec[1]);
    // spdf_tool.translateAndDeform(trans_vec[2]);
  } while(--time);

  time = 20;
  do {
    spdf_tool.translateAndDeform(trans_vec[1]);
    // spdf_tool.translateAndDeform(trans_vec[1]);
    // spdf_tool.translateAndDeform(trans_vec[2]);
  } while(--time);

  // vf_deformer->saveModel("/home/wegatron/tmp/sphere_final.obj");
}

int main(int argc, char *argv[])
{
  // testPlane();
  testSphere();
  return 0;
}
