//
// Created by peteflo on 2/29/16.
//

#include <iostream>
#include "WaypointInterpolator.h"
#include <time.h>

void testWaypointInterpolatorFourSegments(WaypointInterpolator waypoint_interpolator) {
    std::cout << "Testing with waypoints" << std::endl;

    Eigen::MatrixXd waypoints = Eigen::MatrixXd(4,5);
    waypoints << -1, 2, -3, 2, 3,     // Initialize A. The elements can also be
            4, 5, 6, 4, -1,    // matrices, which are stacked along cols
            1, 3.9, 4.0, 2.0, -1.3,
            0, 0.1, -0.1, 0, 0;
    std::cout << "Using waypoints: " << waypoints << std::endl;
    waypoint_interpolator.setWayPoints(waypoints);

    Eigen::VectorXd current_velocities = Eigen::VectorXd(4);
    current_velocities << 1, 3, 0.1, -1;
    std::cout << "Using current velocities: " << current_velocities << std::endl;

    waypoint_interpolator.setCurrentVelocities(current_velocities);
    waypoint_interpolator.setTausWithHeuristic();
    waypoint_interpolator.computeQuadSplineWithFixedTimeSegments();
    Eigen::MatrixXd currentDerivs = waypoint_interpolator.getCurrentDerivativesOfQuadSpline();

    for (int i = 0; i < 13; i ++) {
        Eigen::MatrixXd currentDerivs = waypoint_interpolator.getCurrentDerivativesOfQuadSpline();
        std::cout << "Current derivs are: " << std::endl << currentDerivs << std::endl;
        sleep(1);
    }
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
    for (int i = 0; i < 10; i ++) {
        Eigen::MatrixXd currentDerivs = waypoint_interpolator.getCurrentDerivativesOfQuadSpline();
        std::cout << "Current derivs are: " << currentDerivs << std::endl;
        sleep(1);
    }
    return;
}

int main() {

    std::cout << "Testing WaypointInterpolator Class" << std::endl;

    WaypointInterpolator waypoint_interpolator = WaypointInterpolator();
    testWaypointInterpolatorFourSegments(waypoint_interpolator);
    //testWaypointInterpolatorOneSegment(waypoint_interpolator);

    return 0;
}