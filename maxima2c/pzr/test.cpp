#include<iostream>

#include "collision_energy.h"

using namespace std;

int main(int argc, char *argv[])
{
  double x[3] = {0.5,0.5,0};
  double c[3] = {0,0,0};

  double r = 1.0;

  double energy = 0.0;
  if(inball(x,c,r)) {
    energy = collisionEnergy(x, c, r);
  }

  cout << energy << endl;

  return 0;
}
