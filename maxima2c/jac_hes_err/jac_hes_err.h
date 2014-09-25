#ifndef JAC_HES_ERR_H
#define JAC_HES_ERR_H

#include <Eigen/Dense>
#include <Eigen/Sparse>

template<typename val_type>
val_type maxFabsArray(const val_type *data, const int size);

template<typename val_type>
val_type maxFabsSparseMat(const Eigen::SparseMatrix<val_type> &mat);

template<typename EnergyType>
double graErr(EnergyType &energy, Eigen::VectorXd &x);

template<typename EnergyType>
double hesErr(EnergyType &energy, Eigen::VectorXd &x);

#endif /* JAC_HES_ERR_H */
