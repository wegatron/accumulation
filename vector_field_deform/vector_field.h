#ifndef VECTOR_FIELD_H
#define VECTOR_FIELD_H

namespace zsw{
  class VectorField final
  {
  public:
    VectorField();
    val(const double *x, double *val);
    void setExFunc(std::shared_ptr<Function> ex_func);
    void setFxFunc(std::shared_ptr<Function> fx_func);
    void setBrFunc(std::shared_ptr<BlendFunc> br_func);
    void setRxFunc(std::shared_ptr<RegionFunc> rx_func);
  private:
    std::shared_ptr<Function> ex_func_;
    std::shared_ptr<Function> fx_func_;
    std::shared_ptr<BlendFunc> br_func_;
    std::shared_ptr<RegionFunc> rx_func_;
  };
}
#endif /* VECTOR_FIELD_H */
