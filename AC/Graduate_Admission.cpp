#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

struct Application {
  int index;
  int ge;
  int gi;
  vector<int> choices;
};

bool compare_func(Application a, Application b)
{
  int sa1 = a.ge + a.gi;
  int sb1 = b.ge + b.gi;

  if(sa1<sb1) { return true; }
  else if(sa1 == sb1) {
    return a.ge < b.ge;
  } else { return false; }
}

int main(int argc, char *argv[])
{
  vector<Application> apps;
  vector<int> quotas;
  vector<vector<int> > res; // accepted applications ind
  int N, M, K;
  cin >> N >> M >> K;

  for (int i=0; i<M; ++i) {
    int tmp;
    cin >> tmp; quotas.push_back(tmp);
  }
  for(int i=0; i<N; ++i) {
    Application tmp_app;
    tmp_app.index = i;
    cin >> tmp_app.ge >> tmp_app.gi;
    for(int j=0; j<K; ++j) {
      int tmp;
      cin >> tmp;
      tmp_app.choices.push_back(tmp);
    }
    apps.push_back(tmp_app);
  }

  sort(apps.begin(), apps.end(), compare_func);
  res.resize(M);
  for(int i=N-1; i>=0; --i) {
    for(int j=0; j<K; ++j) {
      const int ch = apps[i].choices[j];
      const int t_size = res[ch].size();
      if(t_size < quotas[ch]) {
        res[ch].push_back(i);
        break;
      } else if(t_size>0){
        const Application &last_app = apps[(res[ch])[t_size-1]];
        if(last_app.ge == apps[i].ge && last_app.gi == apps[i].gi) {
          res[ch].push_back(i);
          break;
        }
      }
    }
  }

  vector<int> output;
  for(int i=0; i<M; ++i) {
    output.clear();
    for(int j=0; j<res[i].size(); ++j) {
      output.push_back(apps[(res[i])[j]].index);
    }
    if(output.size() > 0) {
      sort(output.begin(), output.end());
      cout << output[0];
      for(int j=1; j<output.size(); ++j) {
        cout << " " << output[j];
      }
    }
    cout << endl;
  }
  return 0;
}
