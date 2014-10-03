#ifndef ZSW_LOG_H
#define ZSW_LOG_H

#include <map>
#include <fstream>
#include <boost/shared_ptr.hpp>

namespace zsw{

  class ZswLog{
  public:
    static boost::shared_ptr<ZswLog> getInstance(){
      if(p_instance == NULL){
         p_instance = boost::shared_ptr<ZswLog>(new ZswLog());
      }
      return p_instance;
    }
    ~ZswLog() {
      ofs.close();
    }
    void log(const std::string &log_type, const std::string& info);
  private:
    ZswLog(){ init(); }
    void init();
    std::ofstream ofs;
    std::map<std::string,int> log_type_map;
    static boost::shared_ptr<ZswLog> p_instance;
  };
  typedef boost::shared_ptr<ZswLog> pZswLog;
}//end of namespace

#endif /*ZSW_LOG_H*/
