#include <iostream>
#include <algorithm>

using namespace std;

bool isOneEditDistance(const string &s, const string &t)
{
  if(abs( (int) (s.size() - t.size()) )  > 1) return false;
  size_t i =0, j=0;
  for(; i<s.size() && j<t.size() && s[i]==t[j]; ++i, ++j);
  if(i==s.size() || j==t.size()) return s.size()!=t.size();
  i+= s.size()>=t.size();
  j+= t.size()>=s.size();
  for(; i<s.size() && j<t.size() && s[i]==t[j]; ++i, ++j);
  return i==s.size();
}

void test()
{
  string str[2];
  cin >> str[0] >> str[1];
  cin.ignore();
  std::cout << str[0] << " : " << str[1] << " " <<  isOneEditDistance(str[0], str[1]) << std::endl;
}

int main(int argc, char *argv[])
{
  size_t t;
  cin >> t;
  while(t--) {
    test();
  }
  return 0;
}
