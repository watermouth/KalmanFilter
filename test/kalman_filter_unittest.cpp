#include "gtest/gtest.h"
#include "kalman_filter.h"

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

class KalmanFilterTest : public testing::Test {
protected:
  void SetUp() { }
  void TearDown() { }
};

TEST_F(KalmanFilterTest, e2_5) {
  using namespace boost::assign;
  vector<double> y;
  y += 17, 16.6, 16.3, 16.1, 17.1, 16.9, 16.8, 17.4, 17.1, 17;
  for (vector<double>::iterator it = y.begin(); it != y.end(); ++it) {
    printf("%1.10f, ", *it);
  }
  cout << endl;

  /// model
  Eigen::MatrixXd F(1,1), V(1,1), G(1,1), W(1,1);
  F(0,0) = 1;
  V(0,0) = 0.25;
  G(0,0) = 1;
  W(0,0) = 25;
  /// plain KF
  KalmanFilter kf(F,V,G,W);
  Eigen::VectorXd m0(1);
  Eigen::MatrixXd C0(1,1);
  m0(0) = 17.2;
  C0(0,0) = 1.0;
  kf.SetInitialDistribution(m0, C0);
  /// filtering
  for (vector<double>::iterator it = y.begin(); it != y.end(); ++it) {
    Eigen::VectorXd yy(1);
    yy << *it; 
    kf.Filtering(yy);
    printf("%1.10f, ", kf.core_->m_(0));
  }
  cout << endl;
  /// R dlm
  cout << "results from DLM package are the following;" << endl; 
  {
    vector<double> y;
    y += 17.00190, 16.60394, 16.30298, 16.10199, 17.09021, 16.90187, 16.80100, 17.39413, 17.10288, 17.00101;
    for (vector<double>::iterator it = y.begin(); it != y.end(); ++it) {
      printf("%1.10f, ", *it);
    }
    cout << endl;
  }
}  

TEST_F(KalmanFilterTest, e2_5_bigObsError) {
  using namespace boost::assign;
  vector<double> y;
  y += 17, 16.6, 16.3, 16.1, 17.1, 16.9, 16.8, 17.4, 17.1, 17;
  for (vector<double>::iterator it = y.begin(); it != y.end(); ++it) {
    printf("%1.10f, ", *it);
  }
  cout << endl;

  /// model
  Eigen::MatrixXd F(1,1), V(1,1), G(1,1), W(1,1);
  F(0,0) = 1;
  V(0,0) = 25; // <<<<<<<<<<<
  G(0,0) = 1;
  W(0,0) = 25;
  /// plain KF
  KalmanFilter kf(F,V,G,W);
  Eigen::VectorXd m0(1);
  Eigen::MatrixXd C0(1,1);
  m0(0) = 17.2;
  C0(0,0) = 1.0;
  kf.SetInitialDistribution(m0, C0);
  /// filtering
  for (vector<double>::iterator it = y.begin(); it != y.end(); ++it) {
    Eigen::VectorXd yy(1);
    yy << *it; 
    kf.Filtering(yy);
    printf("%1.10f, ", kf.core_->m_(0));
  }
  cout << endl;
  /// R dlm
  cout << "results from DLM package are the following;" << endl; 
  {
    vector<double> y;
    y += 17.09804, 16.79844, 16.49159, 16.24971, 16.77518, 16.85232, 16.81998, 17.17845, 17.12997, 17.04964;
    for (vector<double>::iterator it = y.begin(); it != y.end(); ++it) {
      printf("%1.10f, ", *it);
    }
    cout << endl;
  }
}  
