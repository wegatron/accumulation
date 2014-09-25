#ifndef TEST_H
#define TEST_H

//! @param err: 4x4
extern "C" void calc_test_(double *M_out, double *M_in, double *k);

extern "C" void calc_test_jac_(double *M_out, double *M_in, double *k);

extern "C" void calc_test0_(double *M_out, double *M_in);

extern "C" void calc_test0_jac_(double *M_out, double *M_in);


extern "C" void calc_test1_(double *M_out, double *M_in, double *A);

extern "C" void calc_test1_jac_(double *M_out, double *M_in, double *A);

#endif // SO2_H
