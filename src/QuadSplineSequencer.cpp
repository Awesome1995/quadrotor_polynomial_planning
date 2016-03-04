//
// Created by peteflo on 2/5/16.
//

#include "QuadSplineSequencer.h"


void QuadSplineSequencer::resetTimeToZero() {
    using namespace std::chrono;
    this->t1 = high_resolution_clock::now();
};

double QuadSplineSequencer::getTime() {
    using namespace std::chrono;
    this->t2 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    //std::cout << "Time for eval is now " << time_span.count() << std::endl;
    return time_span.count();
}

Eigen::MatrixXd QuadSplineSequencer::getDesiredDerivatives(QuadSpline quad_spline){
    double t = getTime();
    return quad_spline.evalDerivativesAtTime(t);
};

