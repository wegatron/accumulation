#ifndef BILATER_NORMAL_FILTER_H
#define BILATER_NORMAL_FILTER_H

#include <jtflib/mesh/trimesh.h>

namespace zsw
{
  class BilateralNormalFilter final
  {
  public:
    BilateralNormalFilter();
    void filter(jtf::mesh::tri_mesh &trimesh);
  private:
    void filterNormal(jtf::mesh::tri_mesh &trimesh);
    void updateVertex(jtf::mesh::tri_mesh &trimesh);

    // recaculate the trimesh's normal'
    void postProcessing(jtf::mesh::tri_mesh &trimesh);
  };
}

#endif /* BILATER_NORMAL_FILTER_H */
