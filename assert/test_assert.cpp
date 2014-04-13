#include <iostream>
#include <fstream>
#include "assertext.h"

using namespace std;

int main(int argc, char *argv[])
{
  ifstream zsw_if;
  zsw_if.open("exist.txt");
  assert_stream(zsw_if, "exist.txt");
  zsw_if.close();
  return 0;
}

