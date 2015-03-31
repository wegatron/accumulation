#include <iostream>
#include <vector>
#include <queue>

using namespace std;

typedef int DisScalar;

const double infinite=1e7;
struct LinkListNode{
  int dest;
  int next;
  DisScalar dis;
};

class Graph{
public:
  Graph(int n);
  void add2WayEdge(const int v0, const int v1, const DisScalar dis);
  void add1wayEdge(const int start, const int end, const DisScalar dis);
private:
  vector<int> head;
  vector<LinkListNode> buffer;
};

double SPFA(&graph, const int vn, const int ori, const int dest)
{
  queue<int> work_q;
  vector<double> dis(vn);
  work_q.push(ori);
  dis.assign(vn, infinite); dis(ori)=0;
  while (!work_q.empty()) {
    const int tmp_begin=work_q.front(); work_q.pop();
    int node_index=tmp_begin;
    while(node_index!=-1) {
      const LinkListNode &tmp_node=graph[node_index];
      const int tmp_end = tmp_node.dest;
      if (dis[tmp_end] > dis[tmp_begin]+tmp_node.dis) {
        dis[tmp_end] = dis[tmp_begin]+tmp_node.dis;
        work_queue.push(tmp_end); // @todo verify if tmp_end is in queue
      }
      node_index=tmp_node.next;
    }
  }
  return dis[dest];
}

void input2Graph(vector<LinkListNode> &graph, int &vn, int &ori, int &dest)
{
  int t;
  cin >> t >> vn;
  graph.reserve(2*t+vn); graph.resize(vn);
  int tmp_begin, tmp_end, tmp_dis;
  for (int i=0; i<vn; ++i) {
    cin >> tmp_begin >> tmp_end >> tmp_dis;

  }
  for (int i=0;i<t;++i) {

  }
}

int main(int argc, char *argv[])
{
  vector<LinkListNode> graph;
  int vn, ori, dest;
  input2Graph(graph, vn, ori, dest);
  double min_dis = SPFA(graph, vn, ori, dest);
  cout << min_dis << endl;
  return 0;
}
