#include "kalman_filter.h"

namespace my_library {

KalmanFilter::KalmanFilter(const Eigen::MatrixXd &F, const Eigen::MatrixXd &V, const Eigen::MatrixXd &G,
  const Eigen::MatrixXd &W)
  : F_(F), V_(V), G_(G), W_(W)
  , core_(new KalmanFilterCore)
{
  /// モデルの設定
  core_->F_ = &F_; 
  core_->V_ = &V_; 
  core_->G_ = &G_; 
  core_->W_ = &W_; 
}

void KalmanFilter::SetInitialDistribution(const Eigen::VectorXd  m0, const Eigen::MatrixXd C0) {
  core_->SetInitialDistribution(m0, C0);
}

void KalmanFilter::Filtering(const Eigen::VectorXd &y) {
  core_->UpdateDistributions(y);
}

}

