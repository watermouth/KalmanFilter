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
  V = MatrixXd::Zero(4,4);
  W = MatrixXd::Zero(4,4);
  V(0,0) = 1e-6;
  W(0,0) = 25;
  W(2,2) = 0.1;
  W(3,3) = 0.1;
  /// initial distribution
  Eigen::VectorXd m0(4);
  Eigen::MatrixXd C0(4,4);
  m0 << 17.2, 0, 0.5, 0.5;
 
  /// AR2 method test
  AR2 obj;
  cout << C0 << endl << "," << endl;
  obj.G(m0, C0);
  cout << C0 << endl << "," << endl;
  /// 改めて初期化
  C0 = MatrixXd::Identity(4,4);
  C0(1,1) = 0;

  /// EKFの初期化
  ExtendedKalmanFilter<AR2> ekf(V, W);
  ekf.SetInitialDistribution(m0, C0);
/*
  /// online update 
  vector<double> y;
  using namespace boost::assign;
  y += 17, 16.6, 16.3, 16.1, 17.1, 16.9, 16.8, 17.4, 17.1, 17;
  for (vector<double>::iterator it = y.begin(); it != y.end(); ++it) {
    Eigen::VectorXd yy(1);
    yy << *it;
    ekf.Filtering(yy);
    printf("%1.10f, ", ekf.kfcore_->m_(0));
  }
  cout << endl;
*/
}
