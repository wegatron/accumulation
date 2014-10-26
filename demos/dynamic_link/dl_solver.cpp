#include "dl_solver.h"

#include <iostream>

using namespace std;

#if WIN32
#define LIB_PREFIX "lib"
#define LIB_SUFFIX ".so"
#else
#define LIB_PREFIX "lib"
#define LIB_SUFFIX ".so"
#endif

extern "C" {
#include <gmodule.h>
}

static int get_function_point(const char* file_name,const char* fun_name, gpointer *point)
{
  GModule *module = g_module_open (file_name, G_MODULE_BIND_LAZY);
  // printf("[zsw_info] %s:%d get fun: %s from file: %s\n", __FILE__, __LINE__, fun_name, file_name);
  if (!module) {
    fprintf(stderr,"%s\n",g_module_error());
    return 1;
  }
  if (!g_module_symbol (module, fun_name, point)) {
    g_module_close (module);
    fprintf(stderr, "can not find symbol!\n");
    return 2;
  }
  if (point == NULL) {
    g_module_close(module);
    fprintf(stderr, "can nt find point!\n");
    return 3;
  }
  return 0;
}

zsw::BaseSolver *ucreateZswSolver(const char *path)
{
  zsw::BaseSolver* (*createZswSolver)() = NULL;
  std::string filename = path ? path : std::string(LIB_PREFIX) + "solver_pack" + LIB_SUFFIX;
  if( get_function_point(filename.c_str(), "createZswSolver", (gpointer*)&createZswSolver)) {
    std::cerr << "can't get_function_point createZswSolver from file "
              << filename
              << std::endl;
    return NULL;
  }
  return createZswSolver();
}
