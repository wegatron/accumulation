#include <iostream>
#include <memory>
#include <list>

using namespace std;

const int dis_map[9][9] = {
  {0,1,2,1,2,3,2,3,4},
  {1,0,1,2,1,2,3,2,3},
  {2,1,0,3,2,1,4,3,2},
  {1,2,3,0,1,2,1,2,3},
  {2,1,2,1,0,1,2,1,2},
  {3,2,1,2,1,0,3,2,1},
  {2,3,4,1,2,3,0,1,2},
  {3,2,3,2,1,2,1,0,1},
  {4,3,2,3,2,1,2,1,0}
};

const int start_state[9] = {8,3,5,1,2,7,4,6,0};
const int target_state[9] = {1,2,3,4,5,6,7,8,0};

struct Node{
  int state[9];
  int h_dis;
  int g_dis;
  shared_ptr<Node> pre;
};

int cal_h(const int from[], const int target[]) {
  int h_dis = 0;
  for(int i=0; i<9; ++i) {
    h_dis += dis_map[from[i]][target[i]];
  }
  return h_dis;
}

void insert_ascending(list<shared_ptr<Node> > &node_list, shared_ptr<Node> &new_node)
{
  list<shared_ptr<Node> >::iterator it = node_list.begin();
  while(it!= node_list.end() && (new_node->g_dis+new_node->h_dis > (*it)->g_dis+(*it)->h_dis)) {
    ++it;
  }
  node_list.insert(it, new_node);
}

Node * find_state(list<shared_ptr<Node> > &node_list, const int state[])
{
  for(shared_ptr<Node> &tmp_node : node_list) {
    int i=0;
    for(; i<9; ++i) {
      if(state[i]!=tmp_node->state[i]) break;
    }
    if(i == 9) {
      return tmp_node.get();
    }
  }
  return nullptr;
}

bool check_insert(shared_ptr<Node> &cur, int state[], list<shared_ptr<Node> > &open, list<shared_ptr<Node> > &close)
{
  int i=0;
  for(; i<9; ++i) {
    if(state[i] != target_state[i]) break;
  }
  if(i ==9) {
    shared_ptr<Node> new_node(new Node());
    new_node->h_dis = 0;
    new_node->g_dis = cur->g_dis+1;
    new_node->pre = cur;
    copy(state, state+9, new_node->state);
    close.push_back(new_node);
    return true;
  }
  if(find_state(close, state) == nullptr) {
    Node * tmp_node = find_state(open, state);
    if(tmp_node == nullptr) {
      shared_ptr<Node> new_node(new Node());
      new_node->h_dis = cal_h(state, target_state);
      new_node->g_dis = cur->g_dis+1;
      new_node->pre = cur;
      copy(state, state+9, new_node->state);
      insert_ascending(open, new_node);
    } else if(tmp_node->g_dis > cur->g_dis + 1) {
      tmp_node->g_dis = cur->g_dis+1;
      tmp_node->pre = cur;
    }
  }
  return false;
}
bool relax(list<shared_ptr<Node> > &open,
           list<shared_ptr<Node> > &close)
{
  shared_ptr<Node> cur = *(open.begin());
  open.erase(open.begin()); close.push_back(cur);
  int zero_pos = 0;
  for(; cur->state[zero_pos]!=0; ++zero_pos);
  int tmp_state[9]; copy(cur->state, cur->state+9, tmp_state);
  // cur move up, down, left, right
  if(zero_pos>2) {
    // move up
    swap(tmp_state[zero_pos], tmp_state[zero_pos-3]);
    if(check_insert(cur,tmp_state, open, close)) return true;
    swap(tmp_state[zero_pos], tmp_state[zero_pos-3]);
  }
  if(zero_pos<6) {
    // move down
    swap(tmp_state[zero_pos], tmp_state[zero_pos+3]);
    if(check_insert(cur,tmp_state, open, close)) return true;
    swap(tmp_state[zero_pos], tmp_state[zero_pos+3]);
  }
  if(zero_pos!=0 && zero_pos!=3 && zero_pos!=6) {
    // move left
    swap(tmp_state[zero_pos], tmp_state[zero_pos-1]);
    if(check_insert(cur,tmp_state, open, close)) return true;
    swap(tmp_state[zero_pos], tmp_state[zero_pos-1]);
  }
  if(zero_pos!=2 && zero_pos!=5 && zero_pos!=8) {
    // move right
    swap(tmp_state[zero_pos], tmp_state[zero_pos+1]);
    if(check_insert(cur,tmp_state, open, close)) return true;
    swap(tmp_state[zero_pos], tmp_state[zero_pos+1]);
  }
  return false;
}

int main(int argc, char *argv[])
{
  list<shared_ptr<Node> > open;
  list<shared_ptr<Node> > close;

  shared_ptr<Node> start_node(new Node());
  start_node->h_dis = cal_h(start_state, target_state);
  start_node->g_dis = 0; start_node->pre = nullptr;
  copy(start_state, start_state+9, start_node->state);

  open.push_back(start_node);
  while(!open.empty()) {
    if(relax(open, close)) break;
  }
  shared_ptr<Node> tmp_node = close.back();
  do {
    for(int i=0; i<9; ++i) {
      cout << tmp_node->state[i] << " ";
    }
    cout << endl;
    tmp_node = tmp_node->pre;
  } while(tmp_node!=nullptr);
  return 0;
}
