#ifndef INCLUDE_KALMAN_FILTER_CORE_H_
#define INCLUDE_KALMAN_FILTER_CORE_H_

#include <Eigen/Dense>

namespace my_library{

///
/// 素朴に数式をそのまま実装したカルマン・フィルタ
/// 平滑化は未対応
///
/// 参考文献 : Rによるベイジアン動的線型モデル
/// 
struct KalmanFilterCore {
  /// フィルタ分布の平均値ベクトル 
  Eigen::VectorXd m_;
  /// フィルタ分布の分散共分散行列
  Eigen::MatrixXd C_;
  /// 状態の1期先予測分布の平均値ベクトル 
  Eigen::VectorXd a_;
  /// 状態の1期先予測分布の分散共分散行列
  Eigen::MatrixXd R_;
  /// 観測値の1期先予測分布の平均値ベクトル 
  Eigen::VectorXd f_;
  /// 観測値の1期先予測分布の分散共分散行列
  Eigen::MatrixXd Q_;
  /// Kalman gain
  Eigen::MatrixXd KG_;

  /// 観測モデルの係数行列へのポインタ
  const Eigen::MatrixXd *F_;
  /// 観測モデルの誤差項の分散共分散行列へのポインタ
  const Eigen::MatrixXd *V_;
  /// システムモデルの係数行列へのポインタ
  const Eigen::MatrixXd *G_;
  /// システムモデルの誤差項の分散共分散行列へのポインタ
  const Eigen::MatrixXd *W_;

  /// 初期分布の設定
  /// 初期化必須の対象を明示する
  /// \param[in] m0 初期状態分布の期待値
  /// \param[in] C0 初期状態分布の分散共分散行列
  void SetInitialDistribution(const Eigen::VectorXd &m0, const Eigen::MatrixXd &C0) {
    m_ = m0;
    C_ = C0;
  }

  /// 状態の1期先予測
  /// \param[out] out_a 予測分布の期待値 
  /// \param[out] out_R 予測分布の共分散行列
  void predict_state(Eigen::VectorXd &out_a, Eigen::MatrixXd &out_R) const {
    out_a = (*G_) * m_;
    predict_state(out_R);
  }

  /// 状態の1期先予測の共分散行列のみ更新する
  /// \param[out] out_R 予測分布の共分散行列
  void predict_state(Eigen::MatrixXd &out_R)
  const
  {
    out_R = (*G_) * C_ * G_->transpose() + (*W_);
  }

  /// 観測の1期先予測
  /// \param[out] out_f 予測分布の期待値 
  /// \param[out] out_Q 予測分布の共分散行列
  void predict_observation(Eigen::VectorXd &out_f, Eigen::MatrixXd &out_Q) const {
    out_f = (*F_) * a_;
    predict_observation(out_Q);
  }

  /// 観測の1期先予測の共分散行列のみ更新する
  /// \param[out] out_Q 予測分布の共分散行列
  void predict_observation(Eigen::MatrixXd &out_Q) const {
    out_Q = (*F_) * R_ * F_->transpose() + (*V_);
  }

  /// フィルタリング
  /// \param[in] y 観測値
  /// \param[out] out_KG カルマンゲイン 
  /// \param[out] out_f フィルタ分布の期待値 
  /// \param[out] out_Q フィルタ分布の共分散行列
  void filtering(const Eigen::VectorXd &y, Eigen::MatrixXd &out_KG,
    Eigen::VectorXd &out_m, Eigen::MatrixXd &out_C)
  {
    out_KG = R_ * F_->transpose() * Q_.inverse();
    out_m = a_ + out_KG * (y - f_); 
    out_C = R_ - out_KG * (*F_) * R_;
  }

  /// 1期前のフィルタ分布を所与として、
  /// 状態の1期先予測・観測の1期先予測・フィルタリングをまとめて実行し、
  /// 各分布（の期待値と分散）を更新する。
  void UpdateDistributions(const Eigen::VectorXd &y) {
    predict_state(a_, R_);
    predict_observation(f_, Q_);
    filtering(y, KG_, m_, C_); 
  }
};

}

#endif // INCLUDE_KALMAN_FILTER_CORE_H_
