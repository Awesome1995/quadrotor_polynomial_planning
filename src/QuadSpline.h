//
// Created by peteflo on 2/5/16.
//

#ifndef SPLINES_QUADSPLINE_H
#define SPLINES_QUADSPLINE_H


#include "OptimalPiecewisePolynomialGenerator.h"

class QuadSpline {

public:
    OptimalPiecewisePolynomial x_optimal_piecewise_poly;
    OptimalPiecewisePolynomial y_optimal_piecewise_poly;
    OptimalPiecewisePolynomial z_optimal_piecewise_poly;
    OptimalPiecewisePolynomial yaw_optimal_piecewise_poly;

    Eigen::MatrixXd evalDerivativesAtTime(const double t) const;

    double getTotalTime() const;
};


#endif //SPLINES_QUADSPLINE_H
