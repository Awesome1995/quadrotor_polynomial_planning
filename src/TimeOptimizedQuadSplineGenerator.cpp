//
// Created by peteflo on 2/5/16.
//

#include "TimeOptimizedQuadSplineGenerator.h"

OptimalPiecewisePolynomial TimeOptimizedQuadSplineGenerator::GenerateTimeOptimized() {

    int n_segments = 3;
    optimal_piecewise_polynomial_generator.setUpOptimization(n_segments);

    Eigen::VectorXd taus = Eigen::VectorXd(n_segments);
    taus << 0.75, 0.5, 1;

    OptimalPiecewisePolynomial optimal_piecewise_poly = optimal_piecewise_polynomial_generator.GenerateWithFixedTimeSegments(taus);

    std::cout << "COSTS " << optimal_piecewise_poly.costs << std::endl;

    return optimal_piecewise_poly;

}