#ifndef INCLUDE_EXTENDED_KALMAN_FILTER_H
#define INCLUDE_EXTENDED_KALMAN_FILTER_H

#include <Eigen/Dense>
#include <Eigen/Core>
#include "kalman_filter_core.h"

namespace my_library{

template <typename T>
struct EKFModelFunctionBase { 
  template <typename Derived>
  void g(const Eigen::MatrixBase<Derived> &v
    , Eigen::MatrixBase<Derived> &out) {
    get().g_impl(v, out);
  }

  template <typename Derived>
  void G(const Eigen::MatrixBase<Derived> &v
    , Eigen::MatrixBase<Derived> &out) {
    get().G_impl(v, out);
  }

protected:
  T& get() {
    return static_cast<T&>(*this);
  }
  const T& get() const {
    return static_cast<const T&>(*this);
  }
};

struct EKFExample1 : EKFModelFunctionBase<EKFExample1> {
  template <typename Derived>
  void g_impl(const Eigen::MatrixBase<Derived> &m
    , Eigen::MatrixBase<Derived> &out)
  {
    out = m;
  }

  template <typename Derived>
  void G_impl(const Eigen::MatrixBase<Derived> &m
    , Eigen::MatrixBase<Derived> &out)
  {
    /// returns unit matrix whose dimenstion is rows(m) * rows(m)
    //
  }
};

template <typename Fun>
struct ExtendedKalmanFilter{
  /// definition objects of
  /// nonlinear functions and Jacobi matrix functions
  Fun fobj_;
  /// 1$B4|A0$N%U%#%k%?J,I[$r=jM?$H$7$F!"(B
  /// $B>uBV$N(B1$B4|@hM=B,!&4QB,$N(B1$B4|@hM=B,!&%U%#%k%?%j%s%0$r$^$H$a$F<B9T$7!"(B
  /// $B3FJ,I[!J$N4|BTCM$HJ,;6!K$r99?7$9$k!#(B
  void UpdateDistributions(const Eigen::VectorXd &y) {
    filtering(y, KG_, m_, C_); 
  }

private:
  /// API$B$K1F6A$rM?$($J$$$h$&$K$9$k$?$a(B, private$B$K$7$F$*$/(B
  KalmanFilterCore kfc_;   
};

}

#endif
