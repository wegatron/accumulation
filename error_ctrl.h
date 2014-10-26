#ifndef ERROR_CTRL_H
#define ERROR_CTRL_H

#include <iostream>
#include <fstream>

namespace zsw{

#define OK 0

#define CHECK_RETCODE(ret_code) do{                             \
    if (ret_code != OK) {                                       \
      std::cerr << "# [ ERROR ] Error occured in " << __FILE__ << __LINE__  \
                << " as ret_code is abnormal!" << std::endl;    \
      return ret_code;                                          \
    }                                                           \
  }while(0)

#define CALL_FUNC(pn) do{                                               \
    std::cout << "# [ Run: " << #pn << " ]" << std::endl;               \
      if(pn != OK){ std::cerr << "# [ ERROR ] " << #pn << std::endl; return __LINE__; } \
  }while(0)

  inline int open_stream(const string &path, const ios_base::openmode mode, fstream &fs) {
    using namespace std;
    fs.open(path, mode);
    if(!fs || !fs.good()) {
      cerr << "Could not open file " << string(path) << endl;
      return 1;
    }
  }
}

#endif /* ERROR_CTRL_H */
