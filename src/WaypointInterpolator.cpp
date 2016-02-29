//
// Created by peteflo on 2/5/16.
//

#include "WaypointInterpolator.h"

void GetWayPoints();

QuadSpline GenerateQuadSplineWithFixedTimeSegments() {

    n_segments = 3;

    // already need to have received waypoints
    optimal_piecewise_polynomial_generator.setUpOptimization(n_segments);

    // With initial tau guess, generate optimal piecewise polynomial
    Eigen::VectorXd initial_taus = Eigen::VectorXd(n_segments);
    initial_taus << 2.0, 2.0, 2.0;

    OptimalPiecewisePolynomial optimal_piecewise_poly = optimal_piecewise_polynomial_generator.GenerateWithFixedTimeSegments(
            initial_taus);

    return optimal_piecewise_poly;
};



private:
OptimalPiecewisePolynomialGenerator optimal_piecewise_polynomial_generator;
int n_segments;
// waypoints
Eigen::VectorXd taus;