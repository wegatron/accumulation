#ifndef _ERROR_H_
#define _ERROR_H_

#include <iostream>

#define GOOD 0
#define FILEOPEN_ERROR 1
#define FILEPARSE_ERROR 2

#define CHECK_RETCODE(ret_code) do{                             \
    if (ret_code != GOOD) {                                     \
      std::cerr << "error occured in " << __FILE__ << __LINE__  \
                << " as ret_code is abnormal!" << std::endl;    \
      return ret_code;                                          \
    }                                                           \
  }while(0)

#endif
