//
// Created by peteflo on 2/5/16.
//

#ifndef SPLINES_QUADSPLINE_H
#define SPLINES_QUADSPLINE_H


#include "OptimalPiecewisePolynomialGenerator.h"

class QuadSpline {

public:
    OptimalPiecewisePolynomial x_piecewise_poly;
    OptimalPiecewisePolynomial y_piecewise_poly;
    OptimalPiecewisePolynomial z_piecewise_poly;
    OptimalPiecewisePolynomial yaw_piecewise_poly;
};


#endif //SPLINES_QUADSPLINE_H
