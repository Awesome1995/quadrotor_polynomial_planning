//
// Created by peteflo on 2/5/16.
//

#ifndef SPLINES_TIMEOPTIMIZEDQUADSPLINEGENERATOR_H
#define SPLINES_TIMEOPTIMIZEDQUADSPLINEGENERATOR_H

#include "OptimalPiecewisePolynomialGenerator.h"

class TimeOptimizedQuadSplineGenerator {

public:
    OptimalPiecewisePolynomial GenerateTimeOptimized();

private:
    OptimalPiecewisePolynomialGenerator optimal_piecewise_polynomial_generator;
    int n_segments;
    Eigen::VectorXd taus;
};


#endif //SPLINES_TIMEOPTIMIZEDQUADSPLINEGENERATOR_H
