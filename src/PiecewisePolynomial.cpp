//
// Created by peteflo on 2/5/16.
//

#include "PiecewisePolynomial.h"
#include <memory>

/**
     * constructor with specified order and number of segments.  initializes all segments to 0
     */
PiecewisePolynomial::PiecewisePolynomial(int order, int n_segments) : taus(n_segments), times(n_segments)
{
    taus.setZero();
    times.setZero();

    for (int i = 0; i < n_segments; i++) {
        polynomials.push_back(std::shared_ptr<Polynomial>(new Polynomial(order)));
    }

}

/**
     * constructor with specified order and specified vector of time segments (taus)
     */
PiecewisePolynomial::PiecewisePolynomial(int order, Eigen::VectorXd taus)
{
    initializeTausAndTimes(taus);
    for (int i = 0; i < taus.size(); i++) {
        polynomials.push_back(std::shared_ptr<Polynomial>(new Polynomial(order)));
    }
}

PiecewisePolynomial::PiecewisePolynomial(std::vector<std::shared_ptr<Polynomial>> const& polys, Eigen::VectorXd taus) {
    initializeTausAndTimes(taus);
    for (int i = 0; i < polys.size(); i++) {
        polynomials.push_back(polys[i]);
    }
}

void PiecewisePolynomial::initializeTausAndTimes(Eigen::VectorXd const& taus) {
    this->taus = taus;
    times = Eigen::VectorXd(taus.size() + 1);
    times[0] = 0.0;
    for (int i = 1; i <= this->taus.size(); i++) {
        times[i] = times[i-1] + this->taus[i-1];
    }
}

void PiecewisePolynomial::findIndexBinarySearch(double t, size_t index_min, size_t index_max, size_t &segment_index,
                                                double &time_within_segment) const
{
    if (t < times[0] ) {
        segment_index = 0;
        time_within_segment = 0.0;
        return;
    }
    if (t > getFinalTime()) {
        segment_index = taus.size()-1;
        time_within_segment = taus[taus.size()-1];
        return;
    }
    else {
        int index_mid = (index_min + index_max) / 2;

        if (times[index_mid] <= t && t < times[index_mid+1]) {
            segment_index = index_mid;
            time_within_segment = t - times[index_mid];
            return;
        }
        else if (times[index_mid] < t) {
            findIndexBinarySearch(t, index_mid + 1, index_max, segment_index, time_within_segment);
        }
        else {
            findIndexBinarySearch(t, index_min, index_mid - 1, segment_index, time_within_segment);
        }

    }

}

void PiecewisePolynomial::findIndexLinearSearch(double t, size_t index_min, size_t index_max, size_t &segment_index,
                                                double &time_within_segment) const
{
    if (t < times[0]) {
        segment_index = 0;
        time_within_segment = 0;
        return;
    }
    else if (t > getFinalTime()) {
        segment_index = taus.size()-1;
        time_within_segment = taus[taus.size()-1];
        return;
    }
    for (size_t i = index_min; i < index_max; i++) {
        if (t >= times[i] && t <= times[i+1]) {
            segment_index = i;
            time_within_segment = t - times[i];
            return;
        }
    }
}

double PiecewisePolynomial::HornersEval(double t)
{

    std::size_t segment_index;
    double time_within_segment;
    findIndexLinearSearch(t, segment_index, time_within_segment);
    return polynomials.at(segment_index)->HornersEval(time_within_segment);

}

double PiecewisePolynomial::evalDerivative(double t, int derivative) const
{
    std::size_t segment_index;
    double time_within_segment;
    findIndexLinearSearch(t, segment_index, time_within_segment);
    segment_index = 0;
//    std::cout << "SEGMENT INDEX found was " << segment_index << " NUM SEGMENTS was " << getNumSegments() << std::endl;
//    std::cout << "I have this many polys " << polynomials.size() << std::endl;
    return polynomials.at(segment_index)->eval(time_within_segment, derivative);
}

void PiecewisePolynomial::setWithCoeffs(Eigen::MatrixXd coeffs)
{
    if (coeffs.rows() != polynomials.at(0)->coeffs.size()) {
        std::cout << "Supplied coefficients matrix doesn't match number of coefficients needed (wrong number of rows)" << std::endl; return; }
    if (coeffs.cols() != getNumSegments()) {
        std::cout  << "Supplied coefficients matrix doesn't match number of time segments (wrong number of columns)" << std::endl; return; }
    for (int i = 0; i < getNumSegments(); i++) {
        polynomials.at(i)->setCoeffs(coeffs.col(i));
    }
}