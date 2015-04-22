#include <iostream>
#include <vector>
#include <stack>
#include <map>

using namespace std;

const int INF = 10000000;

struct City{
  int dis_;
  int hap_;
  int maxium_happiness_;
  double maxium_average_happiness_;
  int choice_cnt_;
  int best_choice_ind_;
  int step;
};

City cts[210];
map<string, int> name2ind;
string ind2name[210];
int g[210][210];

void init_cty(City& t_cty)
{
  t_cty.dis_ = 0;
  t_cty.hap_ = 0;
  t_cty.maxium_happiness_ = 0;
  t_cty.maxium_average_happiness_ = 0;
  t_cty.choice_cnt_ = 1;
  t_cty.best_choice_ind_ = 0;
  t_cty.step = 0;
}

void djstra(const int dest_ind, const int N)
{
  bool visited[210];
  fill(visited, visited+N,false);
  for(int k=0; k<N; ++k) {
    // for(int i=0;i<N;++i) {
    int i; int min_dis = INF;
    int min_index = -1;
    for (i=0; i<N; ++i) {
      if(visited[i]) continue;
      if(cts[i].dis_ < min_dis) {
        min_dis = cts[i].dis_;
        min_index = i;
      }
    }
    i = min_index; visited[i] = true;
    for (int j=0; j<N; ++j) {
      if(cts[i].dis_ + g[i][j] < cts[j].dis_) {
        cts[j].dis_ = cts[i].dis_ + g[i][j];
        cts[j].maxium_happiness_ = cts[j].hap_ + cts[i].maxium_happiness_;
        cts[j].step = cts[i].step + 1;
        cts[j].maxium_average_happiness_ = cts[j].maxium_happiness_*1.0/cts[j].step;
        cts[j].choice_cnt_ = cts[i].choice_cnt_;
        cts[j].best_choice_ind_ = i;
      } else if(cts[i].dis_ + g[i][j] == cts[j].dis_) {
        cts[j].choice_cnt_ += cts[i].choice_cnt_;
        int tmp_maxium_hap = cts[j].hap_ + cts[i].maxium_happiness_;
        double average_maxium_hap = tmp_maxium_hap*1.0/(cts[i].step + 1);
        if(tmp_maxium_hap > cts[j].maxium_happiness_) {
          cts[j].maxium_happiness_ = tmp_maxium_hap;
          cts[j].step = cts[i].step + 1;
          cts[j].maxium_average_happiness_ = cts[j].maxium_happiness_*1.0/cts[j].step;
          cts[j].best_choice_ind_ = i;
        } else if(tmp_maxium_hap == cts[j].maxium_happiness_ &&
                  average_maxium_hap > cts[j].maxium_average_happiness_) {
          cts[j].step = cts[i].step + 1;
          cts[j].maxium_average_happiness_ = average_maxium_hap;
          cts[j].best_choice_ind_ = i;
        }
      }
    }
  }
}

int main(int argc, char *argv[])
{
  int N, K;
  string H;
  cin >> N >> K >> H;

  name2ind[H] = 0;
  ind2name[0] = H;
  init_cty(cts[0]);
  string ct_name;
  int hap;
  for(int i=1;i<N; ++i) {
    cin >> ct_name >> hap;
    name2ind[ct_name] = i;
    ind2name[i] = ct_name;
    cts[i].hap_ = cts[i].maxium_average_happiness_ = cts[i].maxium_happiness_ = hap;
    cts[i].dis_ = INF;
    cts[i].choice_cnt_ = 0;
    cts[i].best_choice_ind_ = 0;
    cts[i].step = 0;
  }

  for(int i=0; i<N; ++i) {
    for (int j=0; j<N; ++j) {
      g[i][j] = INF;
    }
  }

  string ctf, ctt;
  int cost;
  for(int i=0; i<K; ++i) {
    cin >> ctf >> ctt >> cost;
    int f_ind = name2ind[ctf];
    int t_ind = name2ind[ctt];
    g[f_ind][t_ind] = g[t_ind][f_ind] = cost;
  }

  const int dest_ind = name2ind["ROM"];
  djstra(dest_ind, N);
  cout << cts[dest_ind].choice_cnt_ << " " << cts[dest_ind].dis_ << " " << cts[dest_ind].maxium_happiness_ << " " << (int)cts[dest_ind].maxium_average_happiness_ << endl;
  stack<string> st;
  for(int i=dest_ind; i!=0;) {
    i = cts[i].best_choice_ind_;
    st.push(ind2name[i]);
  }
  while(!st.empty()) {
    cout << st.top() << "->";
    st.pop();
  }
  cout << "ROM" << endl;
  return 0;
}
