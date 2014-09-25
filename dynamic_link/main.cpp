#include <iostream>
#include "dl_solver.h"

using namespace std;

int main(int argc, char *argv[])
{
  zsw::BaseSolver *solver = ucreateZswSolver();
  solver->solve();
  delete solver;
  return 0;
}
