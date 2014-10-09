#include <iostream>
#include <cmath>

#include <assert.h>
#include "kd_tree.h"


using namespace std;
using namespace zsw;

int main(int argc, char *argv[])
{
  double val[2][6] = {{2,4,8,5,9,7}, {3,7,1,4,6,2}};
  const int dim =2;
  const int size =6;
  const double *ptr[] = {&val[0][0], &val[1][0]};

  vector<KdTreeNode> kdtree;
  int root = makeKdTree(ptr, size, dim, kdtree);
  cout << "preOrderTraverse:" << endl;
  preOrderTraverse(kdtree, root);

  cout << "\ninOrderTraverse:" << endl;
  inOrderTraverse(kdtree, root);
  cout << endl;
  int nearest = -1;
  double dis = 1e10;
  double search_point[2] ={2, 4.5};
  nearestSearch(kdtree, root, search_point, nearest, dis);
  assert(nearest>=0 && nearest<kdtree.size());
  assert(dis>=0);
  cout << "nearest point: (" << kdtree[nearest].p[0] << "," << kdtree[nearest].p[1]
       << ") dis = " << sqrt(dis) << endl;
  return 0;
}
