#include <iostream>

#include <iomanip>

#include <vector>
#include <queue>
#include <map>

using namespace std;

void bfs(const multimap<int, int> &g, int &step, int &cnt)
{
  queue<int> qu;
  qu.push(-1);
  int index = -1;
  int last = -1;
  step = -1;
  cnt = 0;
  while(!qu.empty()) {
    index = qu.front();
    qu.pop();
    pair<multimap<int,int>::const_iterator, const multimap<int,int>::const_iterator> res = g.equal_range(index);
    for(multimap<int,int>::const_iterator it = res.first; it!=res.second; ++it) {
      qu.push(it->second);
    }
    if(index == last) {
      ++step;
      last = qu.back();
      if(!qu.empty()) { cnt = qu.size(); }
    }
  }
  --step;
}


int main(int argc, char *argv[])
{
  double price = 0.0;
  double r = 0.0;
  int num = 0;
  scanf("%d%lf%lf", &num, &price, &r);
  // cin >> num >> price >> r;

  multimap<int, int> g;

  int tmp = 0;
  for (int i=0; i<num; ++i) {
    // cin >> tmp;
    scanf("%d", &tmp);
    g.insert(pair<int, int>(tmp,i));
  }
  int step = -1;
  int cnt = 0;
  bfs(g, step, cnt);
  for (int i=0; i<step; ++i) {
    price += price*r/100.0;
  }
  printf("%.2lf %d\n", price, cnt);
  return 0;
}
