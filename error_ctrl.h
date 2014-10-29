#ifndef ERROR_CTRL_H
#define ERROR_CTRL_H

#include <iostream>
#include <fstream>

#ifndef WIN32
#include <execinfo.h>
#include <unistd.h>
#endif

#define OK 0

#ifdef PRINT_BACKTRACE_ACTIVE
#define PRINT_BACKTRACE() do {                                  \
    void *buffer[300];                                          \
    int nbtrace = backtrace(buffer, 100);                       \
    backtrace_symbols_fd(buffer, nbtrace, STDOUT_FILENO);       \
  }while(0)
#else
#define PRINT_BACKTRACE()
#endif

// err_hnd throw exception, or return __LINE__;

#define CHECK_RETCODE(ret_code, err_hnd) do{                            \
    if (ret_code != OK) {                                               \
      std::cerr << "# [ERROR] Error occured in " << __FILE__ << __LINE__ \
                << " as ret_code is abnormal!" << std::endl;            \
      PRINT_BACKTRACE();                                                \
      err_hnd;                                                          \
    }                                                                   \
  }while(0)

#define CALL_FUNC(pn, err_hnd) do{                                      \
    std::cout << "# [ Run: " << #pn << " ]" << std::endl;               \
    if(pn != OK){ std::cerr << "# [ERROR] " << #pn << std::endl; PRINT_BACKTRACE(); err_hnd; } \
  }while(0)


#define CALL_FUNC2(ptr, pn, err_hnd) do {               \
    ptr = pn;                                           \
    if (ptr == nullptr) {                               \
      std::cerr << "# [ERROR] " << __FILE__ << __LINE__ \
                << #pn << " return null." << endl;      \
        PRINT_BACKTRACE();                              \
        err_hnd;                                        \
    }                                                   \
  }while(0)


#define OPEN_STREAM(path, fs, mode, err_hnd) do {                       \
    fs.open(path, mode);                                                \
    if(!fs || !fs.good()) {                                             \
      std::cerr << "# [ERROR] could not open file " << path <<  "for " << #mode  << __FILE__ << __LINE__ << std::endl; \
        PRINT_BACKTRACE();                                              \
        err_hnd;                                                        \
    }                                                                   \
  }while(0)

#define ASSURE(cond, err_hnd) do {                                     \
    if(!(cond)) {                                                       \
      std::cerr << "# [ERROR] condition " << #cond << " not satisfied!" << std::endl; \
        PRINT_BACKTRACE();                                              \
        err_hnd;                                                        \
    }                                                                   \
  } while(0)

#define ASSURE_MSG(cond, msg, err_hnd) do {     \
    if(!(cond)) {                               \
      zsw::LOG(msg);                            \
      err_hnd;                                  \
    }                                           \
  }while(0)
#endif /* ERROR_CTRL_H */
