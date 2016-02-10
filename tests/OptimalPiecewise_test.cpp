//
// Created by peteflo on 2/9/16.
//

#include <iostream>
#include "OptimalPiecewisePolynomialGenerator.h"


void testOptimalPiecewise(PiecewisePolynomial& myPiecewisePolynomial) {

}

int main() {
    std::cout << "Testing OptimalPiecewisePolynomialGenerator Class" << std::endl;
    OptimalPiecewisePolynomialGenerator myOptimalPiecewisePolynomialGenerator = OptimalPiecewisePolynomialGenerator();

    myOptimalPiecewisePolynomialGenerator.setUpOptimization();
    return 0;
}