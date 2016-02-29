//
// Created by peteflo on 2/29/16.
//

#include <iostream>
#include "WaypointInterpolator.h"

void testWaypointInterpolatorFourSegments(WaypointInterpolator waypoint_interpolator) {
    std::cout << "Testing with waypoints" << std::endl;

    Eigen::MatrixXd waypoints = Eigen::MatrixXd(4,5);
    waypoints << 1, 2, 3, 4, 5,     // Initialize A. The elements can also be
            4, 5, 6, 7, 8,    // matrices, which are stacked along cols
            1, 9, 27, 48, 49,
            120, 130, 140, 150, 160;
    std::cout << "Using waypoints: " << waypoints << std::endl;
    waypoint_interpolator.setWayPoints(waypoints);

    Eigen::VectorXd current_velocities = Eigen::VectorXd(4);
    current_velocities << 1, 3, 0.1, -1;
    std::cout << "Using current velocities: " << current_velocities << std::endl;

    waypoint_interpolator.setCurrentVelocities(current_velocities);
    waypoint_interpolator.setTausWithHeuristic();
    waypoint_interpolator.computeQuadSplineWithFixedTimeSegments();
    Eigen::MatrixXd currentDerivs = waypoint_interpolator.getCurrentDerivativesOfQuadSpline();

    std::cout << "Current derivs are: " << currentDerivs << std::endl;
    return;
}

void testWaypointInterpolatorOneSegment(WaypointInterpolator waypoint_interpolator) {
    std::cout << "Testing with waypoints" << std::endl;

    Eigen::MatrixXd waypoints = Eigen::MatrixXd(4,2);
    waypoints << 1, 2,     // Initialize A. The elements can also be
            4, 8,    // matrices, which are stacked along cols
            1, 49,
            120, 130;
    std::cout << "Using waypoints: " << waypoints << std::endl;
    waypoint_interpolator.setWayPoints(waypoints);

    Eigen::VectorXd current_velocities = Eigen::VectorXd(4);
    current_velocities << 1, 3, 0.1, -1;
    std::cout << "Using current velocities: " << current_velocities << std::endl;

    waypoint_interpolator.setCurrentVelocities(current_velocities);
    waypoint_interpolator.setTausWithHeuristic();
    waypoint_interpolator.computeQuadSplineWithFixedTimeSegments();
    for (int i = 0; i < 100; i ++) {
        Eigen::MatrixXd currentDerivs = waypoint_interpolator.getCurrentDerivativesOfQuadSpline();
        std::cout << "Current derivs are: " << currentDerivs << std::endl;
    }
    return;
}

int main() {

    std::cout << "Testing WaypointInterpolator Class" << std::endl;

    WaypointInterpolator waypoint_interpolator = WaypointInterpolator();
    testWaypointInterpolatorFourSegments(waypoint_interpolator);
    testWaypointInterpolatorOneSegment(waypoint_interpolator);

    return 0;
}