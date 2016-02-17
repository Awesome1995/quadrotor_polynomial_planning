//
// Created by peteflo on 2/16/16.
//

#include <iostream>
#include "TimeOptimizedQuadSplineGenerator.h"
#include <ctime>
#include <ratio>
#include <chrono>


int main() {
    std::cout << "Testing TimeOptimizedQuadSplineGenerator Class" << std::endl;
    using namespace std::chrono;
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    TimeOptimizedQuadSplineGenerator time_optimized_generator = TimeOptimizedQuadSplineGenerator();
    OptimalPiecewisePolynomial opt_piecewise_poly = time_optimized_generator.GenerateTimeOptimized();
    high_resolution_clock::time_point t2 = high_resolution_clock::now();

    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

    std::cout << "It took me " << time_span.count() << " seconds.";
    std::cout << std::endl;
    std::cout << "That's an average of " << time_span.count()/10 << std::endl;

    return 0;
}