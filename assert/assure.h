/**
 * @file   assure.h
 * @author Megatron <wegatron@Megatron-dev>
 * @date   Tue Apr 15 14:26:48 2014
 * 
 * @brief  assure.h is some functions for making sure some value or operation is right.
 *  this will also function in release version.
 * 
 */
#ifndef _ASSURE_H_
#define _ASSURE_H_

#include <iostream>
#include <stdlib.h>

inline void assure_stream(std::ios& zswios, 
  const char* filename = "") {
  using namespace std;
  if(!zswios || !zswios.good()) {
    cerr << "Could not open file " << string(filename) << endl;
    exit(1);
  }
}

inline void assure_stream(std::ios& zswios, 
                          const std::string& filename = "") {
  using namespace std;
  if(!zswios || !zswios.good()) {
    cerr << "Could not open file " << filename << endl;
    exit(1);
  }
}

#endif /*_ASSURE_H_*/
