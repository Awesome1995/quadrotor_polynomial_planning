//
// Created by peteflo on 2/9/16.
//

#include <iostream>
#include "OptimalPiecewisePolynomialGenerator.h"


void testOptimalPiecewise(OptimalPiecewisePolynomialGenerator& my_optimal_piecewise_poly_generator) {
    int n_segments = 3;
    my_optimal_piecewise_poly_generator.setUpOptimization(n_segments);

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

    return 0;
}