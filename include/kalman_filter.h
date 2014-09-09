#ifndef INCLUDE_KALMAN_FILTER_H_
#define INCLUDE_KALMAN_FILTER_H_

#include "kalman_filter_core.h"
#include <boost/scoped_ptr.hpp>

namespace my_library {

///
/// 素朴に数式をそのまま実装したカルマン・フィルタ
/// 平滑化は未対応
/// 時変係数も未対応
///
/// 参考文献 : Rによるベイジアン動的線型モデル
/// 
struct KalmanFilter {
  /// 時不変の観測モデルの係数行列
  Eigen::MatrixXd F_;
  /// 時不変の観測モデルの誤差項の分散共分散行列
  Eigen::MatrixXd V_;
  /// 時不変のシステムモデルの係数行列
  Eigen::MatrixXd G_;
  /// 時不変のシステムモデルの誤差項の分散共分散行列
  Eigen::MatrixXd W_;
  /// 1 step カルマンフィルタ実装本体
  boost::scoped_ptr<KalmanFilterCore> core_;

  /// 時不変モデルに対するコンストラクタ
  KalmanFilter(const Eigen::MatrixXd &F,
               const Eigen::MatrixXd &V,
               const Eigen::MatrixXd &G,
               const Eigen::MatrixXd &W);
  /// 初期分布の設定
  void SetInitialDistribution(const Eigen::VectorXd  m0, const Eigen::MatrixXd C0);
  /// 予測・フィルタリングステップ
  /// \param[in] y 観測値
  void Filtering(const Eigen::VectorXd &y);
};

}

#endif // INCLUDE_KALMAN_FILTER_H_

