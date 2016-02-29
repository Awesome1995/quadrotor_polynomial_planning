//
// Created by peteflo on 2/5/16.
//

#ifndef SPLINES_QUADSPLINESEQUENCER_H
#define SPLINES_QUADSPLINESEQUENCER_H

#include "QuadSpline.h"
#include <ctime>
#include <ratio>
#include <chrono>
#include <iostream>

class QuadSplineSequencer {
public:
    void resetTimeToZero();
    Eigen::MatrixXd getDesiredDerivatives(QuadSpline quad_spline);
private:
    std::chrono::high_resolution_clock::time_point t1;
    std::chrono::high_resolution_clock::time_point t2;
};


#endif //SPLINES_QUADSPLINESEQUENCER_H
