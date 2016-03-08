//
// Created by peteflo on 2/5/16.
//

#ifndef SPLINES_WAYPOINTINTERPOLATOR_H
#define SPLINES_WAYPOINTINTERPOLATOR_H

#include "OptimalPiecewisePolynomialGenerator.h"
#include "QuadSpline.h"
#include "QuadSplineSequencer.h"

class WaypointInterpolator {

public:
    void setWayPoints(const Eigen::MatrixXd waypoints);
    void setCurrentVelocities(const Eigen::VectorXd current_velocities);
    void setCurrentPositions(const Eigen::VectorXd current_positions);
    void setTausWithHeuristic();
    void computeQuadSplineWithFixedTimeSegments();
    Eigen::MatrixXd getCurrentDerivativesOfQuadSpline();
    Eigen::MatrixXd getDerivativesOfQuadSplineAtTime(double t);
    double getTotalTime() const { return quad_spline.getTotalTime();};
    void setDerivativeToMinimize(int derivative_to_minimize);
    void ResetSequencerTimeToZero();

private:
    OptimalPiecewisePolynomialGenerator optimal_piecewise_polynomial_generator;
    int n_segments;
    Eigen::MatrixXd waypoints;
    Eigen::VectorXd current_velocities;
    Eigen::VectorXd taus;
    QuadSpline quad_spline;
    QuadSplineSequencer quad_spline_sequencer;
};

#endif //SPLINES_WAYPOINTINTERPOLATOR_H
