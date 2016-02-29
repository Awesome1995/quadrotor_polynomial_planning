//
// Created by peteflo on 2/5/16.
//

#ifndef SPLINES_WAYPOINTINTERPOLATOR_H
#define SPLINES_WAYPOINTINTERPOLATOR_H

#include "OptimalPiecewisePolynomialGenerator.h"
#include "QuadSpline.h"

class WaypointInterpolator {

public:
    void GetWayPoints();
    OptimalPiecewisePolynomial GenerateTimeOptimized();

private:
    OptimalPiecewisePolynomialGenerator optimal_piecewise_polynomial_generator;
    int n_segments;
    // waypoints
    Eigen::VectorXd taus;
};

#endif //SPLINES_WAYPOINTINTERPOLATOR_H
