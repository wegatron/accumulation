#ifndef DATA_TYPE_H
#define DATA_TYPE_H

#include <list>
#include <set>

namespace zsw
{
  template <typename Scalar>
    class FakeSet
    {
    public:
      typedef typename std::list<Scalar>::const_iterator const_iterator;
      typedef typename std::list<Scalar>::iterator iterator;
      FakeSet() {}
      void initFromSet(const std::set<Scalar> &in_set) {
        data.resize(in_set.size());
        std::copy(in_set.begin(), in_set.end(), data.begin());
      }
      void insert(Scalar val)
      {
        std::list<Scalar>::iterator it = data.begin();
        while(it!=data.end()) {
          if(val == *it) { return; }
          ++it;
        }
        data.insert(it,val);
      }

      void erase(Scalar val) {
        for(std::list<Scalar>::iterator it = data.begin();
            it!=data.end(); ++it) {
          if(*it == val) {
            data.erase(it);
            break;
          }
        }
      }

      const_iterator cbegin() const { return data.cbegin(); }
      const_iterator cend() const { return data.cend(); }
      iterator begin() { return data.begin(); }
      iterator end() { return data.end(); }
      size_t size() const { return data.size(); }
    private:
      std::list<Scalar> data;
    };

  template <typename Scalar>
    class NthVector
    {
    public:
      NthVector(size_t n, boost::function<bool (Scalar, Scalar)> pre_func) {
        pre_func_ = pre_func;
        n_ = n;
      }
      void insert(Scalar v) {
        std::list<Scalar>::reverse_iterator it;
        for(it=data.rbegin(); it!=data.rend();
            ++it) {
          if(!pre_func(v, *it)) { break; }
        }
        data.insert(it.base(), v);
        if(data.size()>n_) { data.pop_back(); }
      }

      size_t size() const { return data.size(); }
      zsw::Scalar getIthVal(size_t ith) {
        if(ith >=n_ ) {
          std::cerr << "can't get " << ith << " elementh, exceed " << n_ << std::endl;
        }
        std::list<Scalar>::iterator it = data.begin();
        for(size_t i=0; i<ith; ++i) { ++it; }
        return *it;
      }
    private:
      size_t n_;
      boost::function<bool (Scalar, Scalar)> pre_func_;
      std::list<Scalar> data;
    };
}

#endif /* DATA_TYPE_H */
