#ifndef GRAPH_H
#define GRAPH_H

#include <vector>

namespace zsw{
  const double infinite=1e7;
  struct LinkListNode{
    int dest;
    int next;
    double dis;
  };

  double SPFA(const std::vector<LinkListNode> &graph, const int ori, const int dest);
}

#endif /* GRAPH_H */
