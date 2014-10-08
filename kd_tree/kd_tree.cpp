#include "kd_tree.h"

#include <iostream>
#include <algorithm>
#include <assert.h>

using namespace std;

double zsw::cacuVariance(const vector<double> &val)
{
  double average=0.0;
  double squared_average=0.0;
  size_t n = val.size();
  for (int i=0; i<n; ++i) {
    average += val[i];
    squared_average += val[i]*val[i];
  }
  return (squared_average - average*average/n)/n;
}

double zsw::cacuRootSplit(const Part &unsolved, KdTreeNode* &root, size_t &split_dim)
{
  size_t size = unsolved.size();

  vector<double> val[2];
  val[0].reserve(size); val[1].reserve(size);
  for(int i=0; i<size; ++i) {
    val[0].push_back(unsolved[i]->p[0]);
    val[1].push_back(unsolved[i]->p[1]);
  }
  double variance0 = cacuVariance(val[0]);
  double variance1 = cacuVariance(val[1]);
  split_dim = (variance0 < variance1);
  sort(val[split_dim].begin(), val[split_dim].end());
  double ret = val[split_dim][size/2];
  root = NULL;
  for (int i=0; i<size; ++i) {
    if (ret == unsolved[i]->p[split_dim]) {
      root = unsolved[i];
      root->split =split_dim;
      break;
    }
  }
  assert(root != NULL);
  return ret;
}

size_t zsw::solve(const pair<Part*, size_t*> &tmp, const KdTreeNode *first_ptr,
             queue<pair<Part*, size_t*> > &unsolved_queue)
{
  Part &unsolved = *(tmp.first);
  size_t size = unsolved.size();
  KdTreeNode* tmp_root = NULL;
  size_t split_dim = -1;
  double val = cacuRootSplit(unsolved, tmp_root, split_dim);

  Part *left = new Part();
  Part *right = new Part();
  for (size_t i=0; i<size; ++i) {
    if(unsolved[i] == tmp_root) continue;
    if(unsolved[i]->p[split_dim] <= val) {
      left->push_back(unsolved[i]);
    } else {
      right->push_back(unsolved[i]);
    }
  }
  if(left->size()>0) {
    unsolved_queue.push(pair<Part*, size_t*>(left, &tmp_root->left));
  } else {
    tmp_root->left=-1;
    delete left;
  }

  if(right->size()>0) {
    unsolved_queue.push(pair<Part*, size_t*>(right, &tmp_root->right));
  } else {
    tmp_root->right=-1;
    delete right;
  }

  if (tmp.second != NULL) {
    *tmp.second = tmp_root - first_ptr;
  }
  delete tmp.first;
  return tmp_root - first_ptr;
}

size_t zsw::makeKdTree(const double* val[], const size_t size,
                  const size_t dim, vector<KdTreeNode> &kdtree)
{
  kdtree.reserve(size);
  kdtree.clear();
  queue< pair<Part*, size_t*> > unsolved_queue;
  Part *init_part = new Part();
  for (size_t i=0; i<size; ++i) {
    kdtree[i] = { {val[0][i], val[1][i]}, -1, -1, -1};
    init_part->push_back(&kdtree[i]);
  }
  size_t root = solve(pair<Part*, size_t*>(init_part, NULL), &kdtree[0], unsolved_queue);
  while(!unsolved_queue.empty()) {
    pair<Part*, size_t*> tmp = unsolved_queue.front();
    unsolved_queue.pop();
    solve(tmp, &kdtree[0], unsolved_queue);
  }
  return root;
}

void zsw::inOrderTraverse(vector<KdTreeNode> &kdtree, size_t root)
{
  size_t left = kdtree[root].left;
  size_t right = kdtree[root].right;
  if(left!=-1) {
    inOrderTraverse(kdtree, left);
  }
  cout << "(" << kdtree[root].p[0] << "," << kdtree[root].p[1] << ": " << kdtree[root].split
       << ") ";
  if(right != -1) {
    inOrderTraverse(kdtree, right);
  }
}

void zsw::preOrderTraverse(vector<KdTreeNode> &kdtree, size_t root)
{
  size_t left = kdtree[root].left;
  size_t right = kdtree[root].right;
  cout << "(" << kdtree[root].p[0] << "," << kdtree[root].p[1] << ": " << kdtree[root].split
       << ") ";
  if (left != -1) {
    preOrderTraverse(kdtree, left);
  }
  if(right != -1) {
    preOrderTraverse(kdtree, right);
  }
}
