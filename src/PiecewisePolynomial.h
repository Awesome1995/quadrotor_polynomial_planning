//
// Created by peteflo on 2/5/16.
//

#ifndef SPLINES_PIECEWISEPOLYNOMIAL_H
#define SPLINES_PIECEWISEPOLYNOMIAL_H

#include "Polynomial.hpp"
#include <memory>


class PiecewisePolynomial {
public:

    /**
     * default constructor, 1 segment of zero time, order 1 with 0 coefficients,
     */
    PiecewisePolynomial() : taus(1), times(1)
    {
        this->taus.setZero();
        this->times.setZero();

        std::vector<Polynomial> polynomials(taus.cols());
        Polynomial poly1;
        polynomials.at(0) = poly1;
    }

    /**
     * constructor with specified order and number of segments.  initializes all segments to 0
     */
    PiecewisePolynomial(int order, int n_segments);

    /**
     * constructor with specified order and number of segments.  initializes all segments to 0
     */
    PiecewisePolynomial(int order, Eigen::VectorXd taus);

    PiecewisePolynomial(std::vector<std::shared_ptr <Polynomial>> const& polys, Eigen::VectorXd taus);

    double eval(double t) {
        return HornersEval(t);
    };

    void findIndexBinarySearch(double t, std::size_t& segment_index, double& time_within_segment) {
        findIndexBinarySearch(t, 0, times.size()-1, segment_index, time_within_segment);
    };
    void findIndexBinarySearch(double t, std::size_t index_min, std::size_t index_max, size_t &segment_index,
                               double &time_within_segment);

    double HornersEval(double t);

    double evalDerivative(double t, int derivative);

    double getFinalTime() {
        if (times.size() > 0) {
            return times[times.size()-1];
        }
        return 0;
    };

    double getNumSegments() {
        return taus.size();
    };

    void setWithCoeffs(Eigen::MatrixXd coeffs);


private:
    Eigen::VectorXd taus;
    Eigen::VectorXd times;
    std::vector<std::shared_ptr<Polynomial>> polynomials;
};


#endif //SPLINES_PIECEWISEPOLYNOMIAL_H
