//
// Created by peteflo on 2/5/16.
//

#include <iostream>
#include "PiecewisePolynomial.h"
#include "OptimalPiecewisePolynomialGenerator.h"
#include "gtest/gtest.h"


TEST(PiecewisePolyTest, NegativeTime) {
    int order = 5;
    Eigen::VectorXd taus(7);
    taus << 0.25, 0.5, 0.25, 0.1, 0.3, 0.4, 0.8;
    PiecewisePolynomial myPiecewisePolynomial = PiecewisePolynomial(order, taus);
    std::size_t segment_index;
    double time_within_segment;
    myPiecewisePolynomial.findIndexBinarySearch(-1.0, segment_index, time_within_segment);
    ASSERT_EQ(segment_index,0);
    ASSERT_LE(std::abs(time_within_segment), 1e-12);
}

TEST(PiecewisePolyTest, TooHighTime) {
    int order = 5;
    Eigen::VectorXd taus(7);
    taus << 0.25, 0.5, 0.25, 0.1, 0.3, 0.4, 0.8;
    PiecewisePolynomial myPiecewisePolynomial = PiecewisePolynomial(order, taus);
    std::size_t segment_index;
    double time_within_segment;
    myPiecewisePolynomial.findIndexBinarySearch(1e3, segment_index, time_within_segment);
    ASSERT_EQ(segment_index, myPiecewisePolynomial.getNumSegments()-1);
    ASSERT_LE(std::abs(time_within_segment - 0.8), 1e-12);
}

TEST(PiecewisePolyTest, TestMiddleIndex) {
    int order = 5;
    Eigen::VectorXd taus(7);
    taus << 0.25, 0.5, 0.25, 0.1, 0.3, 0.4, 0.8;
    PiecewisePolynomial myPiecewisePolynomial = PiecewisePolynomial(order, taus);
    std::size_t segment_index;
    double time_within_segment;
    myPiecewisePolynomial.findIndexBinarySearch(0.9, segment_index, time_within_segment);
    ASSERT_EQ(segment_index, 2);
    ASSERT_LE(std::abs(time_within_segment - 0.15), 1e-12);
}

TEST(PiecewisePolyTest, TestHornersEval) {
    int order = 5;
    Eigen::VectorXd taus(7);
    taus << 0.25, 0.5, 0.25, 0.1, 0.3, 0.4, 0.8;
    PiecewisePolynomial myPiecewisePolynomial = PiecewisePolynomial(order, taus);
    Eigen::MatrixXd coeffs = Eigen::MatrixXd::Ones(order+1,taus.size());
    myPiecewisePolynomial.setWithCoeffs(coeffs);
    double invalidPosition = myPiecewisePolynomial.HornersEval(-1.0);
    double initPosition = myPiecewisePolynomial.HornersEval(0.0);
    ASSERT_LE(std::abs(invalidPosition - 1.0), 1e-12);
    ASSERT_LE(std::abs(invalidPosition - initPosition), 1e-12);
    double middlePosition = myPiecewisePolynomial.HornersEval(1.0);
    double laterPosition = myPiecewisePolynomial.HornersEval(1.3);
    double endPosition = myPiecewisePolynomial.HornersEval(2.6);
    double tooLatePosition = myPiecewisePolynomial.HornersEval(3.0);
    ASSERT_LE(std::abs(endPosition - tooLatePosition ), 1e-12);
}

TEST(PiecewisePolyTest, TestEvalDerivs) {
    //std::cout << myPiecewisePolynomial.evalDerivative(0.0, 0) << std::endl <<
    //myPiecewisePolynomial.evalDerivative(0.0, 1) << std::endl <<
    //myPiecewisePolynomial.evalDerivative(0.0, 2) << std::endl <<
    //myPiecewisePolynomial.evalDerivative(0.0, 3) << std::endl;
    int order = 5;
    Eigen::VectorXd taus(7);
    taus << 0.25, 0.5, 0.25, 0.1, 0.3, 0.4, 0.8;
    PiecewisePolynomial myPiecewisePolynomial = PiecewisePolynomial(order, taus);
    Eigen::MatrixXd coeffs = Eigen::MatrixXd::Ones(order+1,taus.size());
    myPiecewisePolynomial.setWithCoeffs(coeffs);
    ASSERT_LE(std::abs( myPiecewisePolynomial.evalDerivative(0.0, 0) -   myPiecewisePolynomial.HornersEval(0.0)), 1e-12);
    ASSERT_LE(std::abs( myPiecewisePolynomial.evalDerivative(1.378, 0) -   myPiecewisePolynomial.HornersEval(1.378)), 1e-12);
}

TEST(OptimalPiecewiseTest, TestOptimalPiecewiseZeroed) {
    OptimalPiecewisePolynomialGenerator my_optimal_piecewise_poly_generator = OptimalPiecewisePolynomialGenerator();
    int n_segments = 3;
    my_optimal_piecewise_poly_generator.setUpOptimizationTest(n_segments);

    Eigen::VectorXd taus = Eigen::VectorXd(n_segments);
    taus << 0.75, 0.5, 1;

    OptimalPiecewisePolynomial optimal_piecewise_poly = my_optimal_piecewise_poly_generator.GenerateWithFixedTimeSegments(taus);

    ASSERT_NEAR(optimal_piecewise_poly.piecewise_poly.eval(-1), 0, 1e-9);
    ASSERT_NEAR(optimal_piecewise_poly.piecewise_poly.eval(0), 0, 1e-9);
    ASSERT_NEAR(optimal_piecewise_poly.piecewise_poly.eval(0.75), 4, 1e-9);
    ASSERT_NEAR(optimal_piecewise_poly.piecewise_poly.eval(1), 4.68994, 1e-2);
    ASSERT_NEAR(optimal_piecewise_poly.piecewise_poly.eval(100), 10, 1e-9);
    ASSERT_NEAR(optimal_piecewise_poly.piecewise_poly.eval(1000), 10, 1e-9);

    //std::cout << "COSTS " << optimal_piecewise_poly.costs << std::endl << std::endl;
    ASSERT_GE(optimal_piecewise_poly.costs.sum(), 0);
    ASSERT_EQ(optimal_piecewise_poly.costs.size(), 3);

}


TEST(OptimalPiecewiseTest, TestOptimalPiecewiseWithWaypoints) {
    OptimalPiecewisePolynomialGenerator my_optimal_piecewise_poly_generator = OptimalPiecewisePolynomialGenerator();
    int n_segments = 3;
    Eigen::VectorXd waypoints = Eigen::VectorXd(n_segments+1);
    waypoints << 0.1, 0.75, 0.95, 1.8;
    double current_velocity = 2.3;
    my_optimal_piecewise_poly_generator.setUpOptimizationWithWaypoints(waypoints, current_velocity);

    Eigen::VectorXd taus = Eigen::VectorXd(n_segments);
    taus << 0.75, 0.5, 1;

    OptimalPiecewisePolynomial optimal_piecewise_poly = my_optimal_piecewise_poly_generator.GenerateWithFixedTimeSegments(taus);

    ASSERT_NEAR(optimal_piecewise_poly.piecewise_poly.eval(-1), 0.1, 1e-9);
    ASSERT_NEAR(optimal_piecewise_poly.piecewise_poly.eval(0), 0.1, 1e-9);
    ASSERT_NEAR(optimal_piecewise_poly.piecewise_poly.eval(0.75), 0.75, 1e-9);
    ASSERT_NEAR(optimal_piecewise_poly.piecewise_poly.eval(1), 0.741665, 1e-2);
    ASSERT_NEAR(optimal_piecewise_poly.piecewise_poly.eval(100), 1.8, 1e-9);
    ASSERT_NEAR(optimal_piecewise_poly.piecewise_poly.eval(1000), 1.8, 1e-9);

    //std::cout << "COSTS " << optimal_piecewise_poly.costs << std::endl;
    ASSERT_GE(optimal_piecewise_poly.costs.sum(), 0);
    ASSERT_EQ(optimal_piecewise_poly.costs.size(), 3);
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    std::cout << "Running Splines Tests" << std::endl;
    return RUN_ALL_TESTS();
}