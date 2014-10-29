#include <stdexcept>

#include "../error_ctrl.h"

using namespace std;

#define OK 0

int func2()
{
  return 1;
}

int main(int argc, char *argv[])
{
  CALL_FUNC(func2(), throw runtime_error("this is run time error"));
  cout << func2() << endl;
  return 0;
}
