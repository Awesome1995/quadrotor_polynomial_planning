//
// Created by peteflo on 2/29/16.
//

#include <iostream>
#include "WaypointInterpolator.h"

void testWaypointInterpolator(WaypointInterpolator waypoint_interpolator) {
    std::cout << "Testing with waypoints" << std::endl;

    Eigen::MatrixXd waypoints = Eigen::MatrixXd(4,5);
    waypoints << 1, 2, 3, 4, 5,     // Initialize A. The elements can also be
            4, 5, 6, 7, 8,    // matrices, which are stacked along cols
            7, 8, 9, 10, 11,
            12, 13, 14, 15, 16;
    std::cout << "Using waypoints: " << waypoints << std::endl;
    waypoint_interpolator.setWayPoints(waypoints);

    Eigen::VectorXd current_velocities = Eigen::VectorXd(4);
    std::cout << "Using current velocities: " << current_velocities << std::endl;

    waypoint_interpolator.setCurrentVelocities(current_velocities);
    waypoint_interpolator.setTausWithHeuristic();
    waypoint_interpolator.computeQuadSplineWithFixedTimeSegments();
//    Eigen::MatrixXd currentDerivs = waypoint_interpolator.getCurrentDerivativesOfQuadSpline();
//
//    std::cout << "Current derivs are: " << currentDerivs << std::endl;
    return;
}

int main() {

    std::cout << "Testing WaypointInterpolator Class" << std::endl;

    WaypointInterpolator waypoint_interpolator = WaypointInterpolator();
    testWaypointInterpolator(waypoint_interpolator);

    return 0;
}