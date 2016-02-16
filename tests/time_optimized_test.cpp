//
// Created by peteflo on 2/16/16.
//

#include <iostream>
#include "TimeOptimizedQuadSplineGenerator.h"


int main() {
    std::cout << "Testing TimeOptimizedQuadSplineGenerator Class" << std::endl;
    TimeOptimizedQuadSplineGenerator time_optimized_generator = TimeOptimizedQuadSplineGenerator();
    OptimalPiecewisePolynomial opt_piecewise_poly = time_optimized_generator.GenerateTimeOptimized();

    return 0;
}