#ifndef KD_TREE_H
#define KD_TREE_H

#include <vector>
#include <queue>

namespace zsw{
  struct KdTreeNode{
    double p[2];
    std::size_t left;
    std::size_t right;
    std::size_t split;
  };

  typedef std::vector<KdTreeNode*> Part;

  double cacuVariance(const std::vector<double> &val);

  double cacuRootSplit(const Part &unsolved, KdTreeNode* &root, std::size_t &split_dim);

  std::size_t solve(const std::pair<Part*, std::size_t*> &tmp, const KdTreeNode *first_ptr,
               std::queue<std::pair<Part*, std::size_t*> > &unsolved_queue);

  std::size_t makeKdTree(const double* val[], const std::size_t size,
                    const std::size_t dim, std::vector<KdTreeNode> &kdtree);

  void inOrderTraverse(std::vector<KdTreeNode> &kdtree, std::size_t root);

  void preOrderTraverse(std::vector<KdTreeNode> &kdtree, std::size_t root);
}


#endif /* KD_TREE_H */
