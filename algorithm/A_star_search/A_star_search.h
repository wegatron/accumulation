#ifndef A_STAR_SEARCH_H
#define A_STAR_SEARCH_H

#include <memory>

template<typename StateT, typename scalar, typename InfoT>
struct A_star_node{
  StateT state;
  scalar g_dis;
  scalar h_dis;
  InfoT info;
  std::shared_ptr<A_star_node> pre;
};

template<typename StateT, typename scalar, typename InfoT>
  bool operator<(A_star_node<StateT, scalar, InfoT> &node_a, A_star_node<StateT, scalar, InfoT> &node_b)
{
  return node_a.g_dis + node_a.h_dis < node_b.g_dis + node_b.h_dis;
}

template<typename StateT, typename scalar, typename InfoT>
  class A_star_search{
 public:
  bool relax()
  {
    std::cerr << __FILE__ << __LINE__ << " function " << __FUNCTION__ << " haven't implemented! " << std::endl;
    return true;
  }
  void search()
  {
    while(!open.empty()) {
      if(relax()) return true;
    }
    return false;
  }
  void init(StateT &start_state, StateT &end_state)
  {
  }

 private:
  void insert_ascending(std::list<std::shared_ptr<A_star_node<StateT, scalar, InfoT> > > &node_list,
                        std::shared_ptr<A_star_node<StateT, scalar, InfoT> > &insert_node)
  {
  }
  std::list<std::shared_ptr<A_star_node<StateT, scalar, InfoT> > > open;
  std::list<std::shared_ptr<A_star_node<StateT, scalar, InfoT> > > close;
};

#endif /* A_STAR_SEARCH_H */
