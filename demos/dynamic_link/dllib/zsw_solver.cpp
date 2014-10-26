#include "zsw_solver.h"

#include <iostream>

using namespace std;

void zsw::ZswSolver::solve()
{
  cout << "hell solver!" << endl;
}


extern "C" {
  zsw::BaseSolver *createZswSolver()
  {
    return new zsw::ZswSolver();
  }
}
