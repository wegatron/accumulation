#include <iostream>
#include <sstream>
#include <algorithm>
#include <string.h>
#include <list>

using namespace std;

class Solution {
public:
  struct Op{
    int left;
    int right;
    char op;
  };
  vector<int> diffWaysToCompute(string input) {
    std::vector<int> ret;
    std::vector<Op> data;
    int val=resolveInput(input, data);
    if(data.size()==0) {
      ret.push_back(val);
      return ret;
    }
    // for(Op &tmp_op : data) {
    //   std::cerr << tmp_op.left << tmp_op.op << tmp_op.right << std::endl;
    //    }

    for(size_t i=0; i<data.size(); ++i) {
      recursiveCalc(i,data,ret);
    }
    // std::cerr << "result:" << std::endl;
    // for(int val : ret) {
    //   std::cerr << val << std::endl;
    // }
    return ret;
  }

private:
  int calcUpdate(const size_t idx, std::vector<Op> &data) {
    int val = 0;
    switch(data[idx].op) {
    case '+':
      val=data[idx].left+data[idx].right;
      break;
    case '-':
      val=data[idx].left-data[idx].right;
      break;
    case '*':
      val=data[idx].left*data[idx].right;
      break;
    default:
      break;
    }
    if(idx>0) {
      data[idx-1].right=val;
    }
    if(idx<data.size()-1) {
      data[idx+1].left=val;
    }
    data.erase(data.begin()+idx);
    return val;
  }
  void recursiveCalc(size_t cur, std::vector<Op> data, std::vector<int> &ret)
  {
    int val=calcUpdate(cur,data);
    if(data.size()==0) {
      ret.push_back(val);
      return;
    }
    for(size_t i=max(cur,(size_t)1)-1; i<data.size(); ++i) {
      recursiveCalc(i,data,ret);
    }
  }
  int resolveInput(const string &input, std::vector<Op> &data)
  {
    stringstream ss(input);
    bool input_number=true;
    char input_op=' ';
    int num_pre=0;
    int num_cur=0;
    while(!ss.eof()) {
      if(input_number) {
        ss>>num_cur;
        if(input_op!=' '){
          Op tmp_op={num_pre, num_cur, input_op};
          data.push_back(tmp_op);
        }
        num_pre=num_cur;
      } else {
        ss>>input_op;
      }
      input_number=!input_number;
    }
    return num_cur;
  }
};


class Solution {
public:
  vector<int> diffWaysToCompute(string input) {
    vector<int> result;
    int size = input.size();
    for (int i = 0; i < size; i++) {
      char cur = input[i];
      if (cur == '+' || cur == '-' || cur == '*') {
        // Split input string into two parts and solve them recursively
        vector<int> result1 = diffWaysToCompute(input.substr(0, i));
        vector<int> result2 = diffWaysToCompute(input.substr(i+1));
        for (auto n1 : result1) {
          for (auto n2 : result2) {
            if (cur == '+')
              result.push_back(n1 + n2);
            else if (cur == '-')
              result.push_back(n1 - n2);
            else
              result.push_back(n1 * n2);
          }
        }
      }
    }
    // if the input string contains only number
    if (result.empty())
      result.push_back(atoi(input.c_str()));
    return result;
  }
};


int main(int argc, char *argv[])
{
  Solution slu;
  slu.diffWaysToCompute("2*3-4*5");
  return 0;
}
