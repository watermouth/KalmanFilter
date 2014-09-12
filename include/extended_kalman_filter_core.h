#ifndef INCLUDE_EXTENDED_KALMAN_FILTER_CORE_H
#define INCLUDE_EXTENDED_KALMAN_FILTER_CORE_H

#include <Eigen/Dense>
#include <Eigen/Core>

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
struct ExtendedKalmanFilterCore{
  /// definition objects of
  /// nonlinear functions and Jacobi matrix functions
  Fun function_objects;
};

}

#endif
