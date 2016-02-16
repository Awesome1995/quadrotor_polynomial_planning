//
// Created by peteflo on 2/9/16.
//

#include <iostream>
#include "OptimalPiecewisePolynomialGenerator.h"


void testOptimalPiecewise(PiecewisePolynomial& myPiecewisePolynomial) {

}

int main() {
    std::cout << "Testing OptimalPiecewisePolynomialGenerator Class" << std::endl;
    OptimalPiecewisePolynomialGenerator my_optimal_piecewise_poly = OptimalPiecewisePolynomialGenerator();

    my_optimal_piecewise_poly.setUpOptimization();
    return 0;
}