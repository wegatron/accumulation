#include <iostream>
#include <iterator>
#include <algorithm>

using namespace std;

typedef ostream_iterator<int> ostream_itr;

int is_insert_sort(const int len, const int *ori, const int *curr)
{
  int index = len-1;
  for (; index>0 && ori[index]==curr[index]; --index);
  // assert(i!=0);
  for (int i=0; i<index; ++i) {
    if(curr[i]>curr[i+1]) { index = -1; break; }
  }
  return index;
}

int cal_step(const int len, const int *curr)
{
  int step = 2; // step =1 impossible
  for(; step<=len; step*=2) {
    // cout << "step: " << step << endl;
    for (int i=0; i<len; i+=step) {
      for (int j=i; j<i+step-1 && j<len-1; ++j) {
        if(curr[j]>curr[j+1]) { return step; }
      }
    }
  }
  // assert (false);
  return step;
}

int main(int argc, char *argv[])
{
  int len;
  int ori[110], curr[110];

  cin >> len;
  for (int i=0; i<len; ++i) {
    cin >> ori[i];
  }
  for (int i=0; i<len; ++i) {
    cin >> curr[i];
  }

  int index = is_insert_sort(len,ori, curr);
  if(index != -1) {
    cout << "Insertion Sort" << endl;
    sort(curr, curr+index+2);
  } else {
    cout << "Merge Sort" << endl;
    int step = cal_step(len, curr);
    for (int i=0; i<len; i+=step) {
      int end_index = min(i+step, len);
      sort(curr+i, curr+end_index);
    }
  }
  copy(curr, curr+len-1, ostream_itr(cout, " "));
  cout << curr[len-1] << endl;
  return 0;
}
