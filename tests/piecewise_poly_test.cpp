//
// Created by peteflo on 2/5/16.
//

#include <iostream>
#include "PiecewisePolynomial.h"




void testNegativeTime(PiecewisePolynomial& myPiecewisePolynomial) {
    std::size_t segment_index;
    double time_within_segment;
    myPiecewisePolynomial.findIndexBinarySearch(-1.0, segment_index, time_within_segment);
    assert(segment_index==0);
    assert(std::abs(time_within_segment) < 1e-12);
}

void testTooHighTime(PiecewisePolynomial& myPiecewisePolynomial) {
    std::size_t segment_index;
    double time_within_segment;
    myPiecewisePolynomial.findIndexBinarySearch(1e3, segment_index, time_within_segment);
    assert(segment_index==myPiecewisePolynomial.getNumSegments());
    assert(std::abs(time_within_segment - 0.8) < 1e-12);
}

void testMiddleIndex(PiecewisePolynomial& myPiecewisePolynomial) {
    std::size_t segment_index;
    double time_within_segment;
    myPiecewisePolynomial.findIndexBinarySearch(0.9, segment_index, time_within_segment);
    assert(segment_index==2);
    assert(std::abs(time_within_segment - 0.15) < 1e-12);
}

void testHornersEval(PiecewisePolynomial& myPiecewisePolynomial) {
    double initPosition = myPiecewisePolynomial.HornersEval(-1.0);
    assert(std::abs(initPosition - 0.0) < 1e-12);
}



int main() {
    std::cout << "Testing Piecewise Polynomial Class" << std::endl;

    int order = 5;
    Eigen::VectorXd taus(7);
    taus << 0.25, 0.5, 0.25, 0.1, 0.3, 0.4, 0.8;

    PiecewisePolynomial myPiecewisePolynomial = PiecewisePolynomial(order, taus);

    testNegativeTime(myPiecewisePolynomial);
    testTooHighTime(myPiecewisePolynomial);
    testMiddleIndex(myPiecewisePolynomial);
    testHornersEval(myPiecewisePolynomial);

    return 0;

}