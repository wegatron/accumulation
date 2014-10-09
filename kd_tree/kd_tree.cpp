#include "kd_tree.h"

#include <iostream>
#include <algorithm>
#include <stack>
#include <assert.h>

#define MAX_DIS 1e10;

using namespace std;
//using namespace zsw;

double zsw::cacuVariance(const vector<double> &val)
{
  double average=0.0;
  double squared_average=0.0;
  int n = val.size();
  for (int i=0; i<n; ++i) {
    average += val[i];
    squared_average += val[i]*val[i];
  }
  return (squared_average - average*average/n)/n;
}

double zsw::cacuRootSplit(const Part &unsolved, KdTreeNode* &root, int &split_dim)
{
  int size = unsolved.size();

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

int zsw::solve(const pair<Part*, int*> &tmp, const KdTreeNode *first_ptr,
             queue<pair<Part*, int*> > &unsolved_queue)
{
  Part &unsolved = *(tmp.first);
  int size = unsolved.size();
  KdTreeNode* tmp_root = NULL;
  int split_dim = -1;
  double val = cacuRootSplit(unsolved, tmp_root, split_dim);

  Part *left = new Part();
  Part *right = new Part();
  for (int i=0; i<size; ++i) {
    if(unsolved[i] == tmp_root) continue;
    if(unsolved[i]->p[split_dim] <= val) {
      left->push_back(unsolved[i]);
    } else {
      right->push_back(unsolved[i]);
    }
  }
  if(left->size()>0) {
    unsolved_queue.push(pair<Part*, int*>(left, &tmp_root->left));
  } else {
    tmp_root->left=-1;
    delete left;
  }

  if(right->size()>0) {
    unsolved_queue.push(pair<Part*, int*>(right, &tmp_root->right));
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

int zsw::makeKdTree(const double* val[], const int size,
                  const int dim, vector<KdTreeNode> &kdtree)
{
  kdtree.clear(); kdtree.resize(size);
  queue< pair<Part*, int*> > unsolved_queue;
  Part *init_part = new Part();
  for (int i=0; i<size; ++i) {
    kdtree[i] = { {val[0][i], val[1][i]}, -1, -1, -1};
    init_part->push_back(&kdtree[i]);
  }
  int root = solve(pair<Part*, int*>(init_part, NULL), &kdtree[0], unsolved_queue);
  while(!unsolved_queue.empty()) {
    pair<Part*, int*> tmp = unsolved_queue.front();
    unsolved_queue.pop();
    solve(tmp, &kdtree[0], unsolved_queue);
  }
  return root;
}

void zsw::inOrderTraverse(vector<KdTreeNode> &kdtree, int root)
{
  int left = kdtree[root].left;
  int right = kdtree[root].right;
  if(left!=-1) {
    inOrderTraverse(kdtree, left);
  }
  cout << "(" << kdtree[root].p[0] << "," << kdtree[root].p[1] << ": " << kdtree[root].split
       << ") ";
  if(right != -1) {
    inOrderTraverse(kdtree, right);
  }
}

void zsw::preOrderTraverse(vector<KdTreeNode> &kdtree, int root)
{
  int left = kdtree[root].left;
  int right = kdtree[root].right;
  cout << "(" << kdtree[root].p[0] << "," << kdtree[root].p[1] << ": " << kdtree[root].split
       << ") ";
  if (left != -1) {
    preOrderTraverse(kdtree, left);
  }
  if(right != -1) {
    preOrderTraverse(kdtree, right);
  }
}

double zsw::cacuSquaredDis(const double *val0, const double *val1)
{
  return (val0[0]-val1[0])*(val0[0]-val1[0]) + (val0[1]-val1[1])*(val0[1]-val1[1]);
}

void zsw::nearestSearch(const vector<KdTreeNode> &kdtree, const int root,
                        double val[], int &nearest, double &squared_dis)
{
  stack<int> path_stack;
  int track = -1;
  int now = root;
  squared_dis = MAX_DIS;
  while(now!= -1 || !path_stack.empty()) {
    if (now != -1) { // new visit node
      path_stack.push(now);
      double tmp_dis = cacuSquaredDis(val, kdtree[now].p);
      if (tmp_dis < squared_dis) {
        nearest = now;
        squared_dis = tmp_dis;
      }

      int dim = kdtree[now].split;
      now = (val[dim]<=kdtree[now].p[dim]) ? kdtree[now].left : kdtree[now].right;
      track = -1;
    }
    else { // traverse to parent
      now = path_stack.top();
      int dim = kdtree[now].split;
      int post_order_index = (val[dim] > kdtree[now].p[dim]) ? kdtree[now].left : kdtree[now].right;
      if ( track != post_order_index &&
           (kdtree[now].p[dim]-val[dim])*(kdtree[now].p[dim]-val[dim])<squared_dis) {
        track = now = post_order_index;
      } else {
        path_stack.pop();
        track = now;
        now = -1;
      }
    }
  }
}
