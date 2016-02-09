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
    assert(segment_index==myPiecewisePolynomial.getNumSegments()-1);
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
    double invalidPosition = myPiecewisePolynomial.HornersEval(-1.0);
    double initPosition = myPiecewisePolynomial.HornersEval(0.0);
    assert(std::abs(invalidPosition - 1.0) < 1e-12);
    assert(std::abs(invalidPosition - initPosition) < 1e-12);
    double middlePosition = myPiecewisePolynomial.HornersEval(1.0);
    double laterPosition = myPiecewisePolynomial.HornersEval(1.3);
    double endPosition = myPiecewisePolynomial.HornersEval(2.6);
    double tooLatePosition = myPiecewisePolynomial.HornersEval(3.0);
    assert(std::abs(endPosition - tooLatePosition ) < 1e-12);
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

    Eigen::MatrixXd coeffs = Eigen::MatrixXd::Ones(order+1,taus.size());
    myPiecewisePolynomial.setWithCoeffs(coeffs);

    testHornersEval(myPiecewisePolynomial);

    return 0;

}