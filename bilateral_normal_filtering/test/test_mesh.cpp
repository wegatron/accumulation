#include <iostream>

#include "jtflib/mesh/trimesh.h"

using namespace std;

int main(int argc, char *argv[])
{
  jtf::mesh::tri_mesh trimesh("/home/wegatron/tmp/plane_input.obj");
  std::pair<size_t, size_t> result = trimesh.ea_->query(137, 9);
  std::cout << result.first << " " << result.second << std::endl;
  return 0;
}
