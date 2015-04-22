#include <iostream>
#include <queue>

using namespace std;

struct position
{
  int l;
  int m;
  int n;
};

bool g[60][1286][128];

int volumn_connected[9876480];
int index = 0;

void bfs(const position& start, const position& max_pos, const int threshold, int &cnt)
{
  if(!g[start.l][start.m][start.n]) { return; }
  queue<position> qu;
  qu.push(start);
  g[start.l][start.m][start.n] = false;
  int cur_cnt = 1;
  while(!qu.empty()) {
    position cur = qu.front(); qu.pop();
    if(cur.n-1>=0 && g[cur.l][cur.m][cur.n-1]) {
      position tmp = {cur.l,cur.m,cur.n-1};
      ++cur_cnt; g[tmp.l][tmp.m][tmp.n] = false;
      // cout << "1@" << tmp.l << ":" << tmp.m << ":" << tmp.n << endl;
      qu.push(tmp);
    } //left

    if(cur.n+1<=max_pos.n && g[cur.l][cur.m][cur.n+1]) {
      position tmp = {cur.l,cur.m,cur.n+1};
      ++cur_cnt; g[tmp.l][tmp.m][tmp.n] = false;
      // cout << "2@" << tmp.l << ":" << tmp.m << ":" << tmp.n << endl;
      qu.push(tmp);
    } //right

    if(cur.m-1>=0 && g[cur.l][cur.m-1][cur.n]) {
      position tmp = {cur.l, cur.m-1, cur.n};
      ++cur_cnt; g[tmp.l][tmp.m][tmp.n] = false;
      // cout << "3@" << tmp.l << ":" << tmp.m << ":" << tmp.n << endl;
      qu.push(tmp);
    } //down
    if(cur.m+1<=max_pos.m && g[cur.l][cur.m+1][cur.n]) {
      position tmp = {cur.l, cur.m+1, cur.n};
      ++cur_cnt; g[tmp.l][tmp.m][tmp.n] = false;
      // cout << "4@" << tmp.l << ":" << tmp.m << ":" << tmp.n << endl;
      qu.push(tmp);
    } //up

    if(cur.l-1>=0 && g[cur.l-1][cur.m][cur.n]) {
      position tmp = {cur.l-1, cur.m, cur.n};
      ++cur_cnt; g[tmp.l][tmp.m][tmp.n] = false;
      // cout << "5@" << tmp.l << ":" << tmp.m << ":" << tmp.n << endl;
      qu.push(tmp);
    } //l-1
    if(cur.l+1<=max_pos.l && g[cur.l+1][cur.m][cur.n]) {
      position tmp = {cur.l+1, cur.m, cur.n};
      ++cur_cnt; g[tmp.l][tmp.m][tmp.n] = false;
      // cout << "6@" << tmp.l << ":" << tmp.m << ":" << tmp.n << endl;
      qu.push(tmp);
    } //l+1
  }
  if(cur_cnt >= threshold) { cnt+= cur_cnt; }
}

int main(int argc, char *argv[])
{
  int M,N,L,T;
  cin>> M >> N >> L >> T;
  for(int i=0; i<L; ++i) {
    for (int j=0; j<M; ++j) {
      for (int k=0; k<N; ++k) {
        cin >> g[i][j][k];
      }
    }
  }
  int cnt = 0;
  position max_pos = {L-1, M-1, N-1};
  for(int i=0; i<L; ++i) {
    for (int j=0; j<M; ++j) {
      for (int k=0; k<N; ++k) {
        position cur = {i,j,k};
        bfs(cur, max_pos, T, cnt);
      }
    }
  }

  cout << cnt << endl;
  return 0;
}
