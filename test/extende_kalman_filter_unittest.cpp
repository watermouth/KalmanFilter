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
}
