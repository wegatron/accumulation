#include "test.h"
#include <iostream>

using namespace std;

int main()
{
	double M[] = {0,1,2,3};
	double k= 10;
	double M_out[4] = {0};
	double M_jac_out[16] = {0};
	

	calc_test0_(&M_out[0], &M[0]);
	calc_test0_jac_(&M_jac_out[0], &M[0]);

	for(size_t i = 0; i < 4; ++i)
	  cerr << M_out[i] << " ";
	cerr << endl;

	for(size_t i = 0; i < 16; ++i)
	  cerr << M_jac_out[i] << " ";
	cerr << endl;


///////////////////////////////////////////////////////////////////
	calc_test_(&M_out[0], &M[0], &k);
	calc_test_jac_(&M_jac_out[0], &M[0], &k);

	for(size_t i = 0; i < 4; ++i)
	  cerr << M_out[i] << " ";
	cerr << endl;

	for(size_t i = 0; i < 16; ++i)
	  cerr << M_jac_out[i] << " ";
	cerr << endl;

////////////////////////////////////////////////////////////////////
	double M2[4] = {1,1,1,1};
	calc_test1_(&M_out[0], &M[0], &M2[0]);
	calc_test1_jac_(&M_jac_out[0], &M[0], &M2[0]);

	for(size_t i = 0; i < 4; ++i)
	  cerr << M_out[i] << " ";
	cerr << endl;

	for(size_t i = 0; i < 16; ++i)
	  cerr << M_jac_out[i] << " ";
	cerr << endl;
	
	return 0;
}
