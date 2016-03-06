//
// Created by peteflo on 2/5/16.
//

#include <iostream>
#include "PiecewisePolynomial.h"
#include "OptimalPiecewisePolynomialGenerator.h"
#include "gtest/gtest.h"
#include "WaypointInterpolator.h"
#include <time.h>
#include <ctime>
#include <ratio>
#include <chrono>


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
    ASSERT_NEAR(optimal_piecewise_poly.piecewise_poly.eval(1.25), 0.95, 1e-2);
    ASSERT_NEAR(optimal_piecewise_poly.piecewise_poly.eval(100), 1.8, 1e-9);
    ASSERT_NEAR(optimal_piecewise_poly.piecewise_poly.eval(1000), 1.8, 1e-9);

    //std::cout << "COSTS " << optimal_piecewise_poly.costs << std::endl;
    ASSERT_GE(optimal_piecewise_poly.costs.sum(), 0);
    ASSERT_EQ(optimal_piecewise_poly.costs.size(), 3);
}

TEST(WaypointInterpolatorTest, TestWithFourWaypoints) {

    std::cout << "Running a test with an actual stopwatch..." << std::endl << "Please wait... (15 seconds)" << std::endl;
    using namespace std::chrono;

    WaypointInterpolator waypoint_interpolator = WaypointInterpolator();

    Eigen::MatrixXd waypoints = Eigen::MatrixXd(4,5);
    waypoints << -1, 2, -3, 2, 3,     // Initialize A. The elements can also be
            4, 5, 6, 4, -1,    // matrices, which are stacked along cols
            1, 3.9, 4.0, 2.0, -1.3,
            0, 0.1, -0.1, 0, 0;
    waypoint_interpolator.setWayPoints(waypoints);

    Eigen::VectorXd current_velocities = Eigen::VectorXd(4);
    current_velocities << 1, 3, 0.1, -1;

    waypoint_interpolator.setCurrentVelocities(current_velocities);
    waypoint_interpolator.setTausWithHeuristic();

    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    waypoint_interpolator.computeQuadSplineWithFixedTimeSegments();
    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

    ASSERT_GE(1.0/time_span.count(), 100); // Assert that we can compute new quad splines at greater than 100 Hz (we only need ~10 Hz, but should be in ~500 Hz range, otherwise something is slow)

    Eigen::MatrixXd currentDerivs = waypoint_interpolator.getCurrentDerivativesOfQuadSpline();
    ASSERT_NEAR(waypoints(0,0), currentDerivs(0,0), 0.1); // Check that trajectory starts near first waypoint

    for (int i = 0; i < 15; i ++) {

        high_resolution_clock::time_point t1 = high_resolution_clock::now();
        Eigen::MatrixXd currentDerivs = waypoint_interpolator.getCurrentDerivativesOfQuadSpline();
        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
        ASSERT_GE(1.0/time_span.count(), 5000); // Assert that we can evaluate our trajectory at greater than 5 kHz (we only need ~100 Hz, but we can do this blazing fast, otherwise something is slow)

        //std::cout << "Current derivs are: " << std::endl << currentDerivs << std::endl;
        ASSERT_LE(currentDerivs(0,2), 4); // Testing to make sure Acceleration isn't too high ever
        ASSERT_LE(currentDerivs(1,2), 4);
        ASSERT_LE(currentDerivs(2,2), 4);
        ASSERT_LE(currentDerivs(3,2), 4);

        double speed = std::sqrt(currentDerivs(0,1) * currentDerivs(0,1) + currentDerivs(1,1) * currentDerivs(1,1) + currentDerivs(2,1) * currentDerivs(2,1));
        std::cout << "SPEED AT TIME " << i << " IS " << speed << std::endl;
        ASSERT_LE(speed, 5); // Testing to make sure we never go faster than 5 m / s
        if (i >= 22) {
            ASSERT_NEAR(waypoints(0,4), currentDerivs(0,0), 0.1); // Make sure we remain at endpoint
            ASSERT_NEAR(waypoints(1,4), currentDerivs(1,0), 0.1);
            ASSERT_NEAR(waypoints(2,4), currentDerivs(2,0), 0.1);
            ASSERT_NEAR(waypoints(3,4), currentDerivs(3,0), 0.1);
        }
        sleep(1);
    }
    return;
}

TEST(WaypointInterpolatorTest, TestWithTwoWaypoints) {

    WaypointInterpolator waypoint_interpolator = WaypointInterpolator();

    Eigen::MatrixXd waypoints = Eigen::MatrixXd(4,2);
    waypoints << 1, 2,     // Initialize A. The elements can also be
            4, 8,    // matrices, which are stacked along cols
            1, 49,
            120, 130;
    waypoint_interpolator.setWayPoints(waypoints);

    Eigen::VectorXd current_velocities = Eigen::VectorXd(4);
    current_velocities << 0, 0, 0, 0;

    waypoint_interpolator.setCurrentVelocities(current_velocities);
    waypoint_interpolator.setTausWithHeuristic();
    waypoint_interpolator.computeQuadSplineWithFixedTimeSegments();

    Eigen::MatrixXd currentDerivs = waypoint_interpolator.getCurrentDerivativesOfQuadSpline();
    ASSERT_NEAR(waypoints(0,0), currentDerivs(0,0), 0.1); // Check that trajectory starts near first waypoint
    std::cout << "Please wait... (15 seconds)" << std::endl;
    for (int i = 0; i < 15; i ++) {
        Eigen::MatrixXd currentDerivs = waypoint_interpolator.getCurrentDerivativesOfQuadSpline();
        double speed = std::sqrt(currentDerivs(0,1) * currentDerivs(0,1) + currentDerivs(1,1) * currentDerivs(1,1) + currentDerivs(2,1) * currentDerivs(2,1));
        ASSERT_LE(speed, 5); // Testing to make sure we never go faster than 5 m / s
        //std::cout << "SPEED AT TIME " << i << " IS " << speed << std::endl;
        sleep(1);
    }

    return;
}


int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    std::cout << "Running Splines Tests" << std::endl;
    return RUN_ALL_TESTS();
}