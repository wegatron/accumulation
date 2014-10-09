#ifndef KD_TREE_H
#define KD_TREE_H

#include <vector>
#include <queue>

namespace zsw{
  struct KdTreeNode{
    double p[2];
    int left;
    int right;
    int split;
  };

  typedef std::vector<KdTreeNode*> Part;

  double cacuVariance(const std::vector<double> &val);

  double cacuRootSplit(const Part &unsolved, KdTreeNode* &root, int &split_dim);

  int solve(const std::pair<Part*, int*> &tmp, const KdTreeNode *first_ptr,
               std::queue<std::pair<Part*, int*> > &unsolved_queue);

  int makeKdTree(const double* val[], const int size,
                    const int dim, std::vector<KdTreeNode> &kdtree);

  void inOrderTraverse(std::vector<KdTreeNode> &kdtree, int root);

  void preOrderTraverse(std::vector<KdTreeNode> &kdtree, int root);

  double cacuSquaredDis(const double *val0, const double *val1);

  void nearestSearch(const std::vector<KdTreeNode> &kdtree, const int root,
                     double val[], int &nearest, double &squared_dis);

  /* void nearestNSearch(const std::vector<KdTreeNode> &kdtree, const int root, */
  /*                     double val[], std::vector<int> &nearest_n, */
  /*                     std::vector<double> &squared_dis_n); */
}


#endif /* KD_TREE_H */
