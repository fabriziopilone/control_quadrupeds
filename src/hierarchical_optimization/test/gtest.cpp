#include "hierarchical_optimization/Task.hpp"
#include <stdio.h>
#include <iostream>

#include <gtest/gtest.h>

using namespace task;


struct Fixture : public ::testing::Test {
  virtual void SetUp() {}

  virtual void TearDown() {
    if (::testing::Test::HasFailure()) {
      std::cerr << "Fixture::TearDown sees failures" << std::endl;
    } else {
      std::cout << "Fixture::TearDown sees NO failures" << std::endl;
    }
  }
};

TEST(solve_test, solve1){
    Eigen::MatrixXd A(1,1);
    Eigen::VectorXd b(1);
    Eigen::MatrixXd D(1,1);
    Eigen::VectorXd f(1);

    A(0,0) = 1;
    b(0) = 0;
    D(0,0) = 1;
    f(0) = 2;
    Task test(A, b, D, f);
    Eigen::VectorXd sol = test.solve_QP();

    EXPECT_LT(sol(0), 1e-5);
}

TEST(solve_test, solve2){
    Eigen::MatrixXd A(1,1);
    Eigen::VectorXd b(1);
    Eigen::MatrixXd D(2,2);
    Eigen::VectorXd f(2);

    A(0,0) = 1;
    b(0) = 0;
    D << 1, 0,
        -1, 0;
    f << -2, -1;
    // x = 0
    // x <= -2
    // x >= 1 -> -x <= -1
    Task test(A, b, D, f);
    EXPECT_ANY_THROW(Eigen::VectorXd sol = test.solve_QP());
}