#include <iostream>
#include "kd_tree.h"

using namespace std;
using namespace zsw;

int main(int argc, char *argv[])
{
  double val[2][6] = {{2,4,5,7,8,9}, {3,5,4,2,1,6}};
  const size_t dim =2;
  const size_t size =6;
  const double *ptr[] = {&val[0][0], &val[1][0]};

  vector<KdTreeNode> kdtree;
  size_t root = makeKdTree(ptr, size, dim, kdtree);
  cout << "preOrderTraverse:" << endl;
  preOrderTraverse(kdtree, root);

  cout << "\ninOrderTraverse:" << endl;
  inOrderTraverse(kdtree, root);
  cout << endl;
  return 0;
}
