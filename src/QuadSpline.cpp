//
// Created by peteflo on 2/5/16.
//

#include "QuadSpline.h"

Eigen::MatrixXd QuadSpline::evalDerivativesAtTime(const double t) const{
    int num_derivatives_to_eval = 4;
    Eigen::MatrixXd derivatives_at_current_time = Eigen::MatrixXd(4, num_derivatives_to_eval );

    for (int i = 0; i<4; i++) {
        derivatives_at_current_time(0,i) = x_optimal_piecewise_poly.piecewise_poly.evalDerivative(t, i);
        derivatives_at_current_time(1,i) = y_optimal_piecewise_poly.piecewise_poly.evalDerivative(t, i);
        derivatives_at_current_time(2,i) = z_optimal_piecewise_poly.piecewise_poly.evalDerivative(t, i);
        derivatives_at_current_time(3,i) = yaw_optimal_piecewise_poly.piecewise_poly.evalDerivative(t, i);
    }

    return derivatives_at_current_time;

};

double QuadSpline::getTotalTime() const {
    return x_optimal_piecewise_poly.piecewise_poly.getFinalTime();
};