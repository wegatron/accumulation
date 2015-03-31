#include <iostream>
#include <fstream>
#include <memory>
#include <list>

#include "A_star_search.h"

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

int start_state[9];
int target_state[9];

struct Node{
  int state[9];
  int g_dis;
  int h_dis;
  char mv;
  shared_ptr<Node> pre;
};

bool state_equal(const int a[], const int b[]) {
  for( int i=0; i<9; ++i) {
    if(a[i]!=b[i]) return false;
  }
  return true;
}

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
    if(state_equal(tmp_node->state, state)) return tmp_node.get();
  }
  return nullptr;
}

bool check_insert(shared_ptr<Node> &cur, int state[], list<shared_ptr<Node> > &open, list<shared_ptr<Node> > &close, char mv)
{
  if(state_equal(state, target_state)) {
    shared_ptr<Node> new_node(new Node());
    new_node->h_dis = 0;
    new_node->g_dis = cur->g_dis+1;
    new_node->pre = cur;
    new_node->mv = mv;
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
      new_node->mv = mv;
      copy(state, state+9, new_node->state);
      insert_ascending(open, new_node);
    } else if(tmp_node->g_dis > cur->g_dis + 1) {
      tmp_node->g_dis = cur->g_dis+1;
      tmp_node->pre = cur;
      tmp_node->mv = mv;
    }
  }
  return false;
}

bool move(shared_ptr<Node> &cur, int tmp_state[], const int zero_pos, list<shared_ptr<Node> > &open,
          list<shared_ptr<Node> > &close)
{
  if(zero_pos>2) {
    // move up
    swap(tmp_state[zero_pos], tmp_state[zero_pos-3]);
    if(check_insert(cur,tmp_state, open, close, 'u')) return true;
    swap(tmp_state[zero_pos], tmp_state[zero_pos-3]);
  }
  if(zero_pos<6) {
    // move down
    swap(tmp_state[zero_pos], tmp_state[zero_pos+3]);
    if(check_insert(cur,tmp_state, open, close,'d')) return true;
    swap(tmp_state[zero_pos], tmp_state[zero_pos+3]);
  }
  if(zero_pos!=0 && zero_pos!=3 && zero_pos!=6) {
    // move left
    swap(tmp_state[zero_pos], tmp_state[zero_pos-1]);
    if(check_insert(cur,tmp_state, open, close,'l')) return true;
    swap(tmp_state[zero_pos], tmp_state[zero_pos-1]);
  }
  if(zero_pos!=2 && zero_pos!=5 && zero_pos!=8) {
    // move right
    swap(tmp_state[zero_pos], tmp_state[zero_pos+1]);
    if(check_insert(cur,tmp_state, open, close,'r')) return true;
    swap(tmp_state[zero_pos], tmp_state[zero_pos+1]);
  }
  return false;
}

bool relax(list<shared_ptr<Node> > &open, list<shared_ptr<Node> > &close)
{
  shared_ptr<Node> cur = *(open.begin());
  open.erase(open.begin()); close.push_back(cur);
  int zero_pos = 0;
  for(; cur->state[zero_pos]!=0; ++zero_pos);
  int tmp_state[9]; copy(cur->state, cur->state+9, tmp_state);
  // cur move up, down, left, right
  return move(cur, tmp_state, zero_pos, open, close);
}

void print_state(int state[]) {
  cout <<  state[0] << state[1] << state[2] << endl;
  cout <<  state[3] << state[4] << state[5] << endl;
  cout <<  state[6] << state[7] << state[8] << endl;
}

int main(int argc, char *argv[])
{
  list<shared_ptr<Node> > open;
  list<shared_ptr<Node> > close;

  string file_path;
  cin >> file_path;
  ifstream ifs(file_path);
  for(int i=0; i<9; ++i) ifs>>start_state[i];
  for(int i=0; i<9; ++i) ifs>>target_state[i];

  shared_ptr<Node> start_node(new Node());
  start_node->h_dis = cal_h(start_state, target_state);
  start_node->g_dis = 0; start_node->pre = nullptr;
  start_node->mv = 'x';
  copy(start_state, start_state+9, start_node->state);

  open.push_back(start_node);
  while(!open.empty()) {
    if(relax(open, close)) break;
  }
  shared_ptr<Node> tmp_node = close.back();
  do {
    print_state(tmp_node->state);
    cout << tmp_node->mv << endl;
    tmp_node = tmp_node->pre;
  } while(tmp_node!=nullptr);
  return 0;
}
