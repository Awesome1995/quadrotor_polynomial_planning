//
// Created by peteflo on 2/5/16.
//

#include "WaypointInterpolator.h"

void WaypointInterpolator::setWayPoints(const Eigen::MatrixXd waypoints) {
    // this for example would be called externally in order to pass in the waypoints matrix
    // format of waypoint matrix should be:
    // number of rows = 4, for x, y, z, yaw
    // number of columns = number of waypoints
    // number of segments = number of columns - 1
    this->waypoints = waypoints;
    this->n_segments = waypoints.cols() - 1;
    quad_spline_sequencer.resetTimeToZero();
};

void WaypointInterpolator::setCurrentVelocities(const Eigen::VectorXd current_velocities) {
    // this for example would be called in order to pass in the current velocities
    // format of velocities vector should be:
    // x_velocity, y_velocity, z_velocity, yaw_velocity
    this->current_velocities = current_velocities;
};

void WaypointInterpolator::setTausWithHeuristic() {
    // currently my heuristic is to take the square root of the euclidean distance

    Eigen::VectorXd taus = Eigen::VectorXd(n_segments);
    Eigen::VectorXd euclidean_distance_xyz = Eigen::VectorXd(n_segments);
    for (int i = 0; i < n_segments; i++) {
        double x1 = waypoints(0,i); double x2 = waypoints(0,i+1);
        double y1 = waypoints(1,i); double y2 = waypoints(1,i+1);
        double z1 = waypoints(2,i); double z2 = waypoints(2,i+1);
        euclidean_distance_xyz(i) = std::sqrt( std::pow(x2-x1, 2) + std::pow(y2-y1, 2) + std::pow(z2-z1, 2) );
        taus(i) = std::sqrt(euclidean_distance_xyz(i));
    }
};


void WaypointInterpolator::computeQuadSplineWithFixedTimeSegments() {
    // generate 4 optimal piecewise polys, one each for x, y, z, yaw
    // store each in the quad spline
    optimal_piecewise_polynomial_generator.setUpOptimizationWithWaypoints(waypoints.row(0), current_velocities(0));
    quad_spline.x_optimal_piecewise_poly = optimal_piecewise_polynomial_generator.GenerateWithFixedTimeSegments(taus);

    optimal_piecewise_polynomial_generator.setUpOptimizationWithWaypoints(waypoints.row(1), current_velocities(1));
    quad_spline.y_optimal_piecewise_poly = optimal_piecewise_polynomial_generator.GenerateWithFixedTimeSegments(taus);

    optimal_piecewise_polynomial_generator.setUpOptimizationWithWaypoints(waypoints.row(2), current_velocities(2));
    quad_spline.z_optimal_piecewise_poly = optimal_piecewise_polynomial_generator.GenerateWithFixedTimeSegments(taus);

    optimal_piecewise_polynomial_generator.setUpOptimizationWithWaypoints(waypoints.row(3), current_velocities(3));
    quad_spline.yaw_optimal_piecewise_poly = optimal_piecewise_polynomial_generator.GenerateWithFixedTimeSegments(taus);
};

Eigen::MatrixXd WaypointInterpolator::getCurrentDerivativesOfQuadSpline() {
    return quad_spline_sequencer.getDesiredDerivatives(quad_spline);
}
