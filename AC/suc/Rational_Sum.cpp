#include <iostream>

using namespace std;

long int maxf(long int n0, long int n1)
{
  if(n0 ==0 || n1 ==0) { return 1; }
  do{
    if(n0<n1) { swap(n0, n1); }
    n0 = n0%n1;
  } while(n0);
  return n1;
}

class RationalNum{
public:
  RationalNum(long int numerator, long int denominator):numerator_(numerator), denominator_(denominator) {
  }
  RationalNum operator + (const RationalNum &ra) const {
    long int numerator = numerator_*ra.denominator_ + ra.numerator_*denominator_;
    long int denominator = ra.denominator_*denominator_;
    RationalNum ret(numerator, denominator);
    ret.ratsimp();
    return ret;
  }
  void ratsimp() {
    const long int max_factor = maxf(numerator_, denominator_);
    numerator_/= max_factor;
    denominator_/=max_factor;
  }

  long int numerator_;
  long int denominator_;
};


int main(int argc, char *argv[])
{
  int N;
  char c;
  cin >> N;

  long int numerator, denominator;
  cin >> numerator >> c >> denominator;
  RationalNum res(numerator, denominator);

  for (int i=1; i<N; ++i) {
    cin >> numerator >> c >> denominator;
    res = res + RationalNum(numerator, denominator);
  }
  res.ratsimp();

  long int integrator = res.numerator_/res.denominator_;
  long int new_numerator = res.numerator_%res.denominator_;
  if(new_numerator!=0 && integrator != 0) {
    cout << integrator << " " << new_numerator << "/" << res.denominator_ << endl;
  } else if(new_numerator!=0 && integrator==0) {
    cout << new_numerator << "/" << res.denominator_ << endl;
  } else if(new_numerator==0) {
    cout << integrator << endl;
  }
  return 0;
}
