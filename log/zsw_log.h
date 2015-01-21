#ifndef ZSW_LOG_H
#define ZSW_LOG_H

#include <fstream>
#include <iostream>
#include <sstream>

#include <memory>
#include <map>
#include <vector>

#define ZSW_LOG_ACTIVE

namespace zsw{
  enum STANDARD_OUTPUT{
    COUT = 1,
    CERR = 2
  };

  class CompositOstream final {
  public:
  CompositOstream(int standard_output=0) :standard_output_(standard_output) {}

    void setStandardOutput(int standard_output) {
      standard_output_ = standard_output;
    }

    void clear() {
      oss_.clear();
    }

    void addOstream(const std::shared_ptr<std::ostream> &os) {
      oss_.push_back(os);
    }

    template <typename T>
      CompositOstream& operator<<(const T& t) {
      for(std::shared_ptr<std::ostream>& os : oss_) {
        (*os) << t;
      }
      if(standard_output_ & STANDARD_OUTPUT::COUT) {
        std::cout << t;
      }
      if(standard_output_ & STANDARD_OUTPUT::CERR) {
        std::cerr << t;
      }
      return *this;
    }

    CompositOstream& operator<<(std::ostream& (*__pf)(std::ostream&))
      {
        for(std::shared_ptr<std::ostream>& os : oss_) {
          __pf(*os);
        }
        if(standard_output_ & STANDARD_OUTPUT::COUT) {
          __pf(std::cout);
        }
        if(standard_output_ & STANDARD_OUTPUT::CERR) {
          __pf(std::cerr);
        }
        return *this;
      }
    ~CompositOstream() {}
  private:
    int standard_output_;
    CompositOstream(const CompositOstream& cls) {}
    std::vector<std::shared_ptr<std::ostream>> oss_;
  };

  inline std::string getTime()
    {
      time_t now = time(0);
      tm *localtm = localtime(&now);
      std::stringstream ss;
      ss << "[" << localtm->tm_year+1900 << "-" << localtm->tm_mon+1 << "-" << localtm->tm_mday
         << "-" << localtm->tm_hour
         << ":" << localtm->tm_min
         << ":" << localtm->tm_sec << "] ";
      return ss.str();
    }

  class ZswLog{
  public:
    static std::shared_ptr<ZswLog> getInstance(const std::string& log_type){
      if(p_instance == NULL){
        p_instance = std::shared_ptr<ZswLog>(new ZswLog());
      }
      p_instance->setCLogtype(log_type);
      return p_instance;
    }

    void setCLogtype(const std::string &clog_type) {
      clog_type_ = clog_type;
    }
    void log(const std::string& info);
    template<typename T>
      CompositOstream& operator << (const T& t)
      {
        os << "# [" << clog_type_ << "] "<< getTime() << t;
        return os;
      }
    CompositOstream& operator<<(std::ostream& (*__pf)(std::ostream&)) {
      os << __pf;
      return os;
    }
  private:
  ZswLog(): isos_ready_(false), os(1),clog_type_("") { init(); }
    void init();

    std::string clog_type_;
    bool isos_ready_;
    CompositOstream os;
    std::map<std::string,int> log_type_map;
    static std::shared_ptr<ZswLog> p_instance;
  };
  typedef std::shared_ptr<ZswLog> pZswLog;

}//end of namespace

#ifdef ZSW_LOG_ACTIVE
#define ZSWLOG(log_type, info) do{                      \
    zsw::pZswLog log = zsw::ZswLog::getInstance(log_type);      \
    log->log(info);                           \
  }while(0)

#define NZSWLOG(log_type) (*zsw::ZswLog::getInstance(log_type))
#else
#define ZSWLOG(log_type, info)
#endif /* ZSW_LOG_ACTIVE */

#endif /*ZSW_LOG_H*/
