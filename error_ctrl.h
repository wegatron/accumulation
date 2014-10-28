#ifndef ERROR_CTRL_H
#define ERROR_CTRL_H

#include <iostream>
#include <fstream>

#define OK 0

#define CHECK_RETCODE(ret_code) do{                                     \
    if (ret_code != OK) {                                               \
      std::cerr << "# [ ERROR ] Error occured in " << __FILE__ << __LINE__ \
                << " as ret_code is abnormal!" << std::endl;            \
      return ret_code;                                                  \
    }                                                                   \
  }while(0)

#define CALL_FUNC(pn) do{                                              \
    std::cout << "# [ Run: " << #pn << " ]" << std::endl;               \
      if(pn != OK){ std::cerr << "# [ ERROR ] " << #pn << std::endl; return __LINE__; } \
  }while(0)




#define CALL_FUNC2(ptr, pn) do {                               \
    ptr = pn;                                                   \
    if (ptr == nullptr) {                                       \
      std::cerr << "# [ ERROR ] " << __FILE__ << __LINE__       \
                << #pn << " return null." << endl;              \
        return __LINE__;                                        \
    }                                                           \
  }while(0)


#define OPEN_STREAM(path, fs, mode) do {                        \
    fs.open(path, mode);                                        \
    if(!fs || !fs.good()) {                                     \
      std::cerr << "Could not open file " << path << std::endl; \
      return __LINE__;                                          \
    }                                                           \
  }while(0)
#endif /* ERROR_CTRL_H */
