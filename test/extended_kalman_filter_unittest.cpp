#include "gtest/gtest.h"
#include "extended_kalman_filter.h"

#include <cstdio>
#include <vector>
#include <algorithm>
#include <numeric>
#include <functional>
#include <iterator>
#include <iostream>
#include <boost/assign.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/lambda/lambda.hpp>

using namespace std;
using namespace my_library;

class ExtendedKalmanFilterTest : public testing::Test {
protected:
  void SetUp() { }
  void TearDown() { }
};

TEST_F(ExtendedKalmanFilterTest, EKFExample1Test){
  EKFExample1 obj;
  using namespace Eigen;
  MatrixXd m1(1,1), m2(1,1);
  m1(0,0) = 1.0;
  obj.g(m1, m2);
  cout << "m1: " << endl;
  cout << m1 << endl;
  cout << "m2: " << endl;
  cout << m2 << endl;
  obj.G(m1,m2);
  cout << "m1: " << endl;
  cout << m1 << endl;
  cout << "m2: " << endl;
  cout << m2 << endl;
}

TEST_F(ExtendedKalmanFilterTest, AR2Test){
  using namespace Eigen;
  MatrixXd V, W;
  V = MatrixXd::Zero(1,1);
  W = MatrixXd::Zero(4,4);
  V(0,0) = 1e-6;
  W(0,0) = 25;
  W(2,2) = 0.1;
  W(3,3) = 0.1;
  /// initial distribution
  Eigen::VectorXd m0(4);
  Eigen::MatrixXd C0(4,4);
  m0 << 17.2, 0, 0.5, 0.5;
  C0 = MatrixXd::Identity(4,4);
  C0(1,1) = 0;

  {
    /// EKFの初期化
    ExtendedKalmanFilter<AR2> ekf_temp(V, W);
    ekf_temp.SetInitialDistribution(m0, C0);
  //  ekf_temp.G_ = MatrixXd::Identity(4,4);
  //  ekf_temp.F_ = MatrixXd::Identity(1,4);
    /// one step filtering
    Eigen::VectorXd a, f;
    Eigen::MatrixXd R, Q;
    cout << "one step predict_state " << endl;
    a = Eigen::MatrixXd::Identity(4,1);
    R = Eigen::MatrixXd::Identity(4,4);
    ekf_temp.predict_state(a, R);
    cout << a << endl;
    cout << R << endl;
    cout << "one step predict_observation " << endl;
//    ekf_temp.kfcore_->a_ = a;
//    ekf_temp.kfcore_->R_ = R;
    f = Eigen::MatrixXd::Identity(1,1);
    Q = Eigen::MatrixXd::Identity(1,1);
    ekf_temp.predict_observation(f, Q);
    cout << f << endl;
    cout << Q << endl;
    cout << "one step filtering " << endl;
//    ekf_temp.kfcore_->f_ = f;
//    ekf_temp.kfcore_->Q_ = Q;
    Eigen::VectorXd y(1); y << 1;
    ekf_temp.kfcore_->filtering(y, ekf_temp.kfcore_->KG_,
      ekf_temp.kfcore_->m_, ekf_temp.kfcore_->C_
    );
  }

  /// EKFの初期化
  ExtendedKalmanFilter<AR2> ekf(V, W);
  ekf.SetInitialDistribution(m0, C0);
  /// online update 
  cout << "m0, C0" << endl;
  cout << ekf.kfcore_->m_ << ",, " << endl;
  cout << ekf.kfcore_->C_ << ".. " << endl;
  vector<double> y;
  using namespace boost::assign;
  y += 17, 16.6, 16.3, 16.1, 17.1, 16.9, 16.8, 17.4, 17.1, 17;
  for (vector<double>::iterator it = y.begin(); it != y.end(); ++it) {
    Eigen::VectorXd yy(1);
    yy(0) = *it;
    ekf.Filtering(yy);
//    cout << ekf.kfcore_->m_ << ", " << endl; 
  }
  cout << endl;
/*
*/
}
