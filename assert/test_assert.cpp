#include <iostream>
#include <fstream>
#include "assure.h"

using namespace std;

int main(int argc, char *argv[])
{
  ifstream zsw_if;
  zsw_if.open("exist.txt");
  assure_stream(zsw_if, "exist.txt");
  zsw_if.close();
  return 0;
}

