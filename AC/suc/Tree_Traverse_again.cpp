#include <iostream>
#include <stack>

using namespace std;

struct TreeNode{
  int left;
  int right;
};

TreeNode tn[40];

const int PUSH = 0;
const int POP = 1;

void post_order(int index)
{
  static bool is_first = true;
  if(index == 0) { return; }
  post_order(tn[index].left);
  post_order(tn[index].right);
  if(!is_first) { cout << " "; }
  cout << index; is_first = false;
}

int main(int argc, char *argv[])
{
  int root = 0;
  int last_state = PUSH;
  int last_index = 0;
  int N;
  stack<int> st;
  cin >> N;

  for (int i=0; i<=N; ++i) {
    tn[i].left = tn[i].right = 0;
  }
  string tmp_str;
  int tmp_num;
  for (int i=0; i<2*N; ++i) {
    cin >> tmp_str;
    if(tmp_str == "Push") {
      cin >> tmp_num;
      (last_state == PUSH) ? tn[last_index].left=tmp_num : tn[last_index].right=tmp_num;
      last_state = PUSH;
      last_index = tmp_num;
      st.push(tmp_num);
    } else {
      last_state = POP;
      last_index = st.top();
      st.pop();
    }
  }

  post_order(tn[0].left);
  return 0;
}
