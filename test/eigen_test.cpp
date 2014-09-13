#include "gtest/gtest.h"
#include <iostream>
#include "Eigen/Dense"

using namespace std;
using namespace Eigen;

TEST(EigenTest, FirstExample){
  using Eigen::MatrixXd;
  MatrixXd m(2,2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  std::cout << m << std::endl;
}

template <typename Derived>
void print_size(const EigenBase<Derived> &b)
{
  std::cout << "size (rows, cols): "
  << b.size() << " (" << b.rows()
  << ", " << b.cols() << ")"
  << std::endl;
}

TEST(EigenTest, EigenBaseUseTest){
  Vector3d v;
  print_size(v);
  print_size<Vector3d>(v);
  print_size(v.asDiagonal());
}

template <typename Derived>
void print_block(const DenseBase<Derived> &b, int x, int y
  , int r, int c)
{
  std::cout << "block: " << b.block(x,y,r,c)
  << std::endl;
}

TEST(EigenTest, DenseBaseUseTest){
  Vector3d v(1,2,3);
  print_block(v, 0, 0, 2, 1);
}

template <typename Derived>
void AddZeroZeroElement(const MatrixBase<Derived> &M
  , MatrixBase<Derived> &out_M)
{
  out_M(0,0) += M(0,0);
  std::cout << "input: " << std::endl << M
  << ", " << std::endl << "output: " << std::endl 
  << out_M
  << std::endl;
} 

TEST(EigenTest, MatrixBaseUseTest){
  Vector3d v(1,2,3), v2(0,0,0);
  AddZeroZeroElement(v, v2); 
  MatrixXd M1(2,2), M2(3,3);
  M1(0,0) = 7;
  M1(0,1) = 8;
  M1(1,0) = 4;
  M1(1,1) = 5;
  M2 = MatrixXd::Identity(3,3);
  AddZeroZeroElement(M1, M2);
}

TEST(EigenTest, InitializationMethod){
  Vector3d v(1,2,3);
  v << 1, 1, 1;
  cout << v << endl << "," << endl;
  v(1,0) = 2;
  v(2) = 0;
  cout << v;
}

