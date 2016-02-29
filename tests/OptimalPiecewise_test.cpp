//
// Created by peteflo on 2/9/16.
//

#include <iostream>
#include "OptimalPiecewisePolynomialGenerator.h"


void testOptimalPiecewise(OptimalPiecewisePolynomialGenerator& my_optimal_piecewise_poly_generator) {
    int n_segments = 3;
    my_optimal_piecewise_poly_generator.setUpOptimizationTest(n_segments);

    Eigen::VectorXd taus = Eigen::VectorXd(n_segments);
    taus << 0.75, 0.5, 1;

    OptimalPiecewisePolynomial optimal_piecewise_poly = my_optimal_piecewise_poly_generator.GenerateWithFixedTimeSegments(taus);

    std::cout << optimal_piecewise_poly.piecewise_poly.eval(-1) << std::endl;
    std::cout << optimal_piecewise_poly.piecewise_poly.eval(0) << std::endl;
    std::cout << optimal_piecewise_poly.piecewise_poly.eval(0.75) << std::endl;
    std::cout << optimal_piecewise_poly.piecewise_poly.eval(1) << std::endl;
    std::cout << optimal_piecewise_poly.piecewise_poly.eval(100) << std::endl;
    std::cout << optimal_piecewise_poly.piecewise_poly.eval(1000) << std::endl;

    std::cout << "COSTS " << optimal_piecewise_poly.costs << std::endl << std::endl;
}

void testOptimalPiecewisewithWaypoints(OptimalPiecewisePolynomialGenerator& my_optimal_piecewise_poly_generator) {

    std::cout << "Testing with externally specified waypoints, current velocity, and taus" << std::endl;

    int n_segments = 3;
    Eigen::VectorXd waypoints = Eigen::VectorXd(n_segments+1);
    waypoints << 0.1, 0.75, 0.95, 1.8;
    double current_velocity = 2.3;
    my_optimal_piecewise_poly_generator.setUpOptimizationWithWaypoints(waypoints, current_velocity);

    Eigen::VectorXd taus = Eigen::VectorXd(n_segments);
    taus << 0.75, 0.5, 1;

    OptimalPiecewisePolynomial optimal_piecewise_poly = my_optimal_piecewise_poly_generator.GenerateWithFixedTimeSegments(taus);

    std::cout << optimal_piecewise_poly.piecewise_poly.eval(-1) << std::endl;
    std::cout << optimal_piecewise_poly.piecewise_poly.eval(0) << std::endl;
    std::cout << optimal_piecewise_poly.piecewise_poly.eval(0.75) << std::endl;
    std::cout << optimal_piecewise_poly.piecewise_poly.eval(1) << std::endl;
    std::cout << optimal_piecewise_poly.piecewise_poly.eval(100) << std::endl;
    std::cout << optimal_piecewise_poly.piecewise_poly.eval(1000) << std::endl;

    std::cout << "COSTS " << optimal_piecewise_poly.costs << std::endl;
}

int main() {
    std::cout << "Testing OptimalPiecewisePolynomialGenerator Class" << std::endl;
    OptimalPiecewisePolynomialGenerator my_optimal_piecewise_poly_generator = OptimalPiecewisePolynomialGenerator();
    testOptimalPiecewise(my_optimal_piecewise_poly_generator);
    testOptimalPiecewisewithWaypoints(my_optimal_piecewise_poly_generator);

    return 0;
}