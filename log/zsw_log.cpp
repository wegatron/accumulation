#include "zsw_log.h"

#include <iostream>
#include <sstream>
#include <string>
#include <time.h>
#include <utility>

using namespace std;
using namespace zsw;

static const string log_path_prefix="/tmp/zswlog";
static const int log_type_size = 1;
static pair<string, int> log_types[] = {
  pair<string, int>("zsw_log",1)
};

pZswLog ZswLog::p_instance;

static string getTime()
{
  time_t now = time(0);
  tm *localtm = localtime(&now);
  stringstream ss;
  ss << "[" << localtm->tm_year+1900 << "-" << localtm->tm_mon+1 << "-" << localtm->tm_mday
     << "-" << localtm->tm_hour
     << ":" << localtm->tm_min
     << ":" << localtm->tm_sec << "] ";
  return ss.str();
}

void zsw::ZswLog::init()
{
  for (int i=0; i<log_type_size; ++i) {
    log_type_map.insert(log_types[i]);
  }

  string file_path = log_path_prefix + ".log";
  ofs.open(file_path.c_str(), std::fstream::app);
  if(!ofs || !ofs.good()) {
    cerr << "can not open log file: " << file_path << "to write!" << endl;
  }
}

void zsw::ZswLog::log(const string &log_type, const string& info)
{
  map<string,int>::const_iterator it = log_type_map.find(log_type);
  if (!ofs || !ofs.good() || it == log_type_map.end() || it->second==0) {
    return;
  }
  cout <<  getTime() << info << endl;
  ofs << getTime() << info << endl;
}
