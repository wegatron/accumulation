#include <iostream>
#include <vector>

using namespace std;

int main(int argc, char *argv[])
{
  int numerator, denominator;
  cin >> numerator >> denominator;
  if(numerator == 0) { cout << 0 << endl;  return 0;}
  int res_int = numerator/denominator;
  vector<int> ints(denominator, 0);
  for(int i=0; i<denominator; ++i) {
    ints[i] = i*10/denominator;
  }
  vector<bool> vs(denominator, false);
  vector<int> float_parts;
  for(int le = numerator%denominator; le!=0 && !vs[le]; ) {
    float_parts.push_back(ints[le]);
    vs[le] = true;
    le = (le*10)%denominator;
  }

  return 0;
}
