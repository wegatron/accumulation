      SUBROUTINE calc_test0(test0,R) 
      DOUBLE PRECISION R(2,*) 
      DOUBLE PRECISION test0(2,*) 
      test0(1,1) = R[1,1]
      test0(1,2) = R[1,2]
      test0(2,1) = R[2,1]
      test0(2,2) = R[2,2]
      END 
      SUBROUTINE calc_test0_jac(test0_jac,R) 
      DOUBLE PRECISION test0_jac(4,*) 
      test0_jac(1,1) = 1
      test0_jac(1,2) = 0
      test0_jac(1,3) = 0
      test0_jac(1,4) = 0
      test0_jac(2,1) = 0
      test0_jac(2,2) = 1
      test0_jac(2,3) = 0
      test0_jac(2,4) = 0
      test0_jac(3,1) = 0
      test0_jac(3,2) = 0
      test0_jac(3,3) = 1
      test0_jac(3,4) = 0
      test0_jac(4,1) = 0
      test0_jac(4,2) = 0
      test0_jac(4,3) = 0
      test0_jac(4,4) = 1
      END 
      SUBROUTINE calc_test1(test1,R,A) 
      DOUBLE PRECISION A(2,*) 
      DOUBLE PRECISION R(2,*) 
      DOUBLE PRECISION test1(2,*) 
      test1(1,1) = R[1,1]+A[1,1]
      test1(1,2) = R[1,2]+A[1,2]
      test1(2,1) = R[2,1]+A[2,1]
      test1(2,2) = R[2,2]+A[2,2]
      END 
      SUBROUTINE calc_test1_jac(test1_jac,R,A) 
      DOUBLE PRECISION test1_jac(4,*) 
      test1_jac(1,1) = 1
      test1_jac(1,2) = 0
      test1_jac(1,3) = 0
      test1_jac(1,4) = 0
      test1_jac(2,1) = 0
      test1_jac(2,2) = 1
      test1_jac(2,3) = 0
      test1_jac(2,4) = 0
      test1_jac(3,1) = 0
      test1_jac(3,2) = 0
      test1_jac(3,3) = 1
      test1_jac(3,4) = 0
      test1_jac(4,1) = 0
      test1_jac(4,2) = 0
      test1_jac(4,3) = 0
      test1_jac(4,4) = 1
      END 
      SUBROUTINE calc_test(test,R,k) 
      DOUBLE PRECISION R(2,*) 
      DOUBLE PRECISION test(2,*) 
      test(1,1) = R[1,1]*k
      test(1,2) = R[1,2]*k
      test(2,1) = R[2,1]*k
      test(2,2) = R[2,2]*k
      END 
      SUBROUTINE calc_test_jac(test_jac,R,k) 
      DOUBLE PRECISION test_jac(4,*) 
      test_jac(1,1) = k
      test_jac(1,2) = 0
      test_jac(1,3) = 0
      test_jac(1,4) = 0
      test_jac(2,1) = 0
      test_jac(2,2) = k
      test_jac(2,3) = 0
      test_jac(2,4) = 0
      test_jac(3,1) = 0
      test_jac(3,2) = 0
      test_jac(3,3) = k
      test_jac(3,4) = 0
      test_jac(4,1) = 0
      test_jac(4,2) = 0
      test_jac(4,3) = 0
      test_jac(4,4) = k
      END 
