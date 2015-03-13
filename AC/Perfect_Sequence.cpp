#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

int zsw_bsearch(const vector<int>& v, int right, int left, const int min_v) // value > min_v
{
  int middle  = 0;
  while(right>left) {
    middle = (right+left)>>1;
    if(v[middle]>min_v) { right = middle; }
    else { left = middle+1; }
  }
  return right;
}

int main(int argc, char *argv[])
{
  int N = 0;
  int p = 0;

  vector<int> v;

  cin >> N >> p;
  for(int i=0; i<N; ++i) {
    int tmp =0; cin >> tmp;
    v.push_back(tmp);
  }
  sort(&v[0], &v[0]+N);
  int start = 0;
  int range_min = v[0];
  long long int range_max = v[0]*p;
  int max_cnt = 1, tmp_cnt = 1;
  for(int index=1; index<N; ++index) {
    if(v[index]>=range_min && v[index]<=range_max) {
      ++tmp_cnt;
      max_cnt = max(tmp_cnt, max_cnt);
    } else if(v[index]>range_max) {
      // binary search
      int min_v = v[index]/p;
      if(v[index]%p) { ++min_v; }
      start = zsw_bsearch(v, index, start, v[index]/p);
      range_min = v[start]; range_max = ((long long int)v[start])* ((long long int)p);
      tmp_cnt = index - start + 1;
    }
  }
  cout << max_cnt << endl;
  return 0;
}
