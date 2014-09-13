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
  /// 観測モデルのJacobi行列
  Eigen::MatrixXd F_;
  /// 観測モデルの誤差項の分散共分散行列
  Eigen::MatrixXd V_;
  /// システムモデルのJacobi行列
  Eigen::MatrixXd G_;
  /// システムモデルの誤差項の分散共分散行列
  Eigen::MatrixXd W_;
  /// definition objects of
  /// nonlinear functions and Jacobi matrix functions
  boost::scoped_ptr<Fun> fobj_ptr_;
  ExtendedKalmanFilter(
    const Eigen::MatrixXd &V,
    const Eigen::MatrixXd &W)
  : V_(V), W_(W), fobj_ptr_(new Fun), kfcore_(new KalmanFilterCore)
  {
    /// pointerの設定
    kfcore_->F_ = &F_; 
    kfcore_->V_ = &V_; 
    kfcore_->G_ = &G_; 
    kfcore_->W_ = &W_; 
  }
  /// 初期分布の設定
  void SetInitialDistribution(
    const Eigen::VectorXd &m0, const Eigen::MatrixXd &C0)
  {
    kfcore_->SetInitialDistribution(m0, C0);
  }
  /// 状態の1期先予測
  /// \post{G_が更新されている}
  /// \param[out] out_a 予測分布の期待値 
  /// \param[out] out_R 予測分布の共分散行列
  void predict_state(
    Eigen::VectorXd &out_a, Eigen::MatrixXd &out_R) const 
  {
    /// 先にJacobi行列の更新
    fobj_ptr_->G(kfcore_->m_, G_);
    /// covariance matrix
    kfcore_->predict_state(out_R);
    /// mean vector
    fobj_ptr_->g(kfcore_->m_, out_a);
  }
  /// 観測の1期先予測
  /// \post{F_が更新されている}
  /// \param[out] out_f 予測分布の期待値 
  /// \param[out] out_Q 予測分布の共分散行列
  void predict_observation(
    Eigen::VectorXd &out_f, Eigen::MatrixXd &out_Q) const
  {
    /// 先にJacobi行列の更新
    fobj_ptr_->F(kfcore_->a_, F_);
    /// covariance matrix
    kfcore_->predict_observation(out_Q);
    /// mean vector
    fobj_ptr_->f(kfcore_->a_, out_f);
  }
  /// 予測・フィルタリングステップ
  /// 1期前のフィルタ分布を所与として、
  /// 状態の1期先予測・観測の1期先予測・フィルタリングをまとめて実行し、
  /// 各分布（の期待値と分散）を更新する。
  /// \param[in] y 観測値
  void Filtering(const Eigen::VectorXd &y)
  {
    predict_state(kfcore_->a_, kfcore_->R_); 
    predict_observation(kfcore_->f_, kfcore_->Q_); 
    kfcore_->filtering(y, kfcore_->KG_, kfcore_->m_, kfcore_->C_);
  }
private:
  /// APIに影響を与えないようにするため, privateにしておく
  boost::scoped_ptr<KalmanFilterCore> kfcore_;   
};

}

#endif
