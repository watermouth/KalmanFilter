#ifndef INCLUDE_EXTENDED_KALMAN_FILTER_H
#define INCLUDE_EXTENDED_KALMAN_FILTER_H

#include <Eigen/Dense>
#include <Eigen/Core>
#include <boost/scoped_ptr.hpp>
#include "kalman_filter_core.h"

namespace my_library{

template <typename T>
struct EKFModelFunctionBase { 
  /// state model
  template <typename Derived>
  void g(const Eigen::MatrixBase<Derived> &v
    , Eigen::MatrixBase<Derived> &out) {
    get().g_impl(v, out);
  }
  /// state model Jacobi matrix
  template <typename Derived>
  void G(const Eigen::MatrixBase<Derived> &v
    , Eigen::MatrixBase<Derived> &out) {
    get().G_impl(v, out);
  }
  /// observation model
  template <typename Derived>
  void f(const Eigen::MatrixBase<Derived> &v
    , Eigen::MatrixBase<Derived> &out) {
    get().f_impl(v, out);
  }
  /// observation model Jacobi matrix
  template <typename Derived>
  void F(const Eigen::MatrixBase<Derived> &v
    , Eigen::MatrixBase<Derived> &out) {
    get().F_impl(v, out);
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
  void g_impl(const Eigen::MatrixBase<Derived> &v
    , Eigen::MatrixBase<Derived> &out)
  {
    out = v;
  }
  template <typename Derived>
  void G_impl(const Eigen::MatrixBase<Derived> &v
    , Eigen::MatrixBase<Derived> &out)
  {
    /// returns unit matrix whose dimenstion is rows(m) * rows(m)
    out = Eigen::MatrixXd::Identity(v.rows(), v.rows());
  }
  template <typename Derived>
  void f_impl(const Eigen::MatrixBase<Derived> &v
    , Eigen::MatrixBase<Derived> &out)
  {
  }

  template <typename Derived>
  void F_impl(const Eigen::MatrixBase<Derived> &v
    , Eigen::MatrixBase<Derived> &out)
  {
  }
  double coeff1_;
};

template <typename Fun>
struct ExtendedKalmanFilter{
  /// $B4QB,%b%G%k$N(BJacobi$B9TNs(B
  Eigen::MatrixXd F_;
  /// $B4QB,%b%G%k$N8m:99`$NJ,;66&J,;69TNs(B
  Eigen::MatrixXd V_;
  /// $B%7%9%F%`%b%G%k$N(BJacobi$B9TNs(B
  Eigen::MatrixXd G_;
  /// $B%7%9%F%`%b%G%k$N8m:99`$NJ,;66&J,;69TNs(B
  Eigen::MatrixXd W_;
  /// definition objects of
  /// nonlinear functions and Jacobi matrix functions
  boost::scoped_ptr<Fun> fobj_ptr_;
  ExtendedKalmanFilter(
    const Eigen::MatrixXd &V,
    const Eigen::MatrixXd &W)
  : V_(V), W_(W), fobj_ptr_(new Fun), kfcore_(new KalmanFilterCore)
  {
    /// pointer$B$N@_Dj(B
    kfcore_->F_ = &F_; 
    kfcore_->V_ = &V_; 
    kfcore_->G_ = &G_; 
    kfcore_->W_ = &W_; 
  }
  /// $B=i4|J,I[$N@_Dj(B
  void SetInitialDistribution(
    const Eigen::VectorXd &m0, const Eigen::MatrixXd &C0)
  {
    kfcore_->SetInitialDistribution(m0, C0);
  }
  /// $B>uBV$N(B1$B4|@hM=B,(B
  /// \post{G_$B$,99?7$5$l$F$$$k(B}
  /// \param[out] out_a $BM=B,J,I[$N4|BTCM(B 
  /// \param[out] out_R $BM=B,J,I[$N6&J,;69TNs(B
  void predict_state(
    Eigen::VectorXd &out_a, Eigen::MatrixXd &out_R) const 
  {
    /// $B@h$K(BJacobi$B9TNs$N99?7(B
    fobj_ptr_->G(kfcore_->m_, G_);
    /// covariance matrix
    kfcore_->predict_state(out_R);
    /// mean vector
    fobj_ptr_->g(kfcore_->m_, out_a);
  }
  /// $B4QB,$N(B1$B4|@hM=B,(B
  /// \post{F_$B$,99?7$5$l$F$$$k(B}
  /// \param[out] out_f $BM=B,J,I[$N4|BTCM(B 
  /// \param[out] out_Q $BM=B,J,I[$N6&J,;69TNs(B
  void predict_observation(
    Eigen::VectorXd &out_f, Eigen::MatrixXd &out_Q) const
  {
    /// $B@h$K(BJacobi$B9TNs$N99?7(B
    fobj_ptr_->F(kfcore_->a_, F_);
    /// covariance matrix
    kfcore_->predict_observation(out_Q);
    /// mean vector
    fobj_ptr_->f(kfcore_->a_, out_f);
  }
  /// $BM=B,!&%U%#%k%?%j%s%0%9%F%C%W(B
  /// 1$B4|A0$N%U%#%k%?J,I[$r=jM?$H$7$F!"(B
  /// $B>uBV$N(B1$B4|@hM=B,!&4QB,$N(B1$B4|@hM=B,!&%U%#%k%?%j%s%0$r$^$H$a$F<B9T$7!"(B
  /// $B3FJ,I[!J$N4|BTCM$HJ,;6!K$r99?7$9$k!#(B
  /// \param[in] y $B4QB,CM(B
  void Filtering(const Eigen::VectorXd &y)
  {
    predict_state(kfcore_->a_, kfcore_->R_); 
    predict_observation(kfcore_->f_, kfcore_->Q_); 
    kfcore_->filtering(y, kfcore_->KG_, kfcore_->m_, kfcore_->C_);
  }
private:
  /// API$B$K1F6A$rM?$($J$$$h$&$K$9$k$?$a(B, private$B$K$7$F$*$/(B
  boost::scoped_ptr<KalmanFilterCore> kfcore_;   
};

}

#endif
