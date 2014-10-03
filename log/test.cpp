#include "zsw_log.h"

using namespace zsw;

int main(int argc, char *argv[])
{
  pZswLog log = ZswLog::getInstance();
  log->log("zsw_log", "good time");
  return 0;
}
