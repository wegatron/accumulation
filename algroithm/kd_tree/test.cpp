#include <iostream>
#include <cmath>

#include <time.h>
#include <stdlib.h>
#include <assert.h>

#include "kd_tree.h"

using namespace std;
using namespace zsw;

void bruteForce2DNearestSearch(const double **ptr, int size, double *search_point,
                             int &nearest, double &squared_dis)
{
  nearest = 0;
  squared_dis = (ptr[0][0] - search_point[0])*(ptr[0][0] - search_point[0]) +
    (ptr[1][0] - search_point[1])*(ptr[1][0] - search_point[1]);
  for (int i=1; i<size; ++i) {
     double tmp_squared_dis =
       (ptr[0][i] - search_point[0])*(ptr[0][i] - search_point[0]) +
    (ptr[1][i] - search_point[1])*(ptr[1][i] - search_point[1]);
     if (tmp_squared_dis < squared_dis) {
       nearest = i;
       squared_dis = tmp_squared_dis;
     }
  }
}

void testNearestSearch()
{
  double val[2][1000];
  const double *ptr[] = {&val[0][0], &val[1][0]};
  const int size = 1000;
  const int dim = 2;
  srand(time(NULL));
  for (int i=0; i<1000; ++i) {
    val[0][i] = (rand()%100000)/1000.0;
    val[1][i] = (rand()%100000)/1000.0;
  }
  double search_point[2];
  search_point[0] = (rand()%100000)/1000.0;
  search_point[1] = (rand()%100000)/1000.0;

  vector<KdTreeNode> kdtree;
  int root = makeKdTree(ptr, 1000, 2, kdtree);
  double dis = 0.0;
  int nearest = -1;
  // cout << "search point is:(" << search_point[0] << "," << search_point[1] << ")" << endl;
  nearestSearch(kdtree, root, search_point, nearest, dis);
  // cout << "kdtree search: (" << kdtree[nearest].p[0] << "," << kdtree[nearest].p[1] << ")"
  //      << "dis: " << dis << endl;
  double real_dis;
  int real_nearest = -1;
  bruteForce2DNearestSearch(ptr, 1000, search_point, real_nearest, real_dis);
  // cout << "brute force: (" << val[0][real_nearest] << "," << val[1][nearest] << ")"
  //      << "dis: " << real_dis << endl;
  assert(real_nearest == nearest);
  assert(fabs(real_dis - dis) < 1e-6);
}

void testNearestSearchManual()
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
}

int main(int argc, char *argv[])
{
  for (int i=0; i<10; ++i)
    testNearestSearch();
  return 0;
}
