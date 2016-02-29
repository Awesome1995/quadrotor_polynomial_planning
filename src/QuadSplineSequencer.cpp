//
// Created by peteflo on 2/5/16.
//

#include "QuadSplineSequencer.h"


void QuadSplineSequencer::resetTimeToZero() {
    using namespace std::chrono;
    this->t1 = high_resolution_clock::now();
};

Eigen::MatrixXd QuadSplineSequencer::currentDesiredDerivatives(QuadSpline quad_spline){
    using namespace std::chrono;
    this->t2 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    return quad_spline.evalDerivativesAtTime(time_span);
};

