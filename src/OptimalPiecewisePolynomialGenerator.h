//
// Created by peteflo on 2/5/16.
//

#ifndef SPLINES_OPTIMALPIECEWISEPOLYNOMIALGENERATOR_H
#define SPLINES_OPTIMALPIECEWISEPOLYNOMIALGENERATOR_H

#include "PiecewisePolynomial.h"


class OptimalPiecewisePolynomialGenerator {
public:

    void setUpOptimization();

    void GenerateWithFixedTimeSegments(const Eigen::VectorXd & taus, const Eigen::VectorXd & der_0,
                                    const Eigen::VectorXd & der_final, const Eigen::VectorXd & der_costs, const Eigen::MatrixXd & intermediate_der,
                                    Polynomial * polys[], Eigen::MatrixXd & opt_ders, Eigen::VectorXd & costs, int intermediate_ders_fixed = 0);
    void setInitialPositionConstrains(const Eigen::VectorXd & initial_derivatives);


private:
    int n_segments;
    int n_derivatives_specified;
    Eigen::VectorXd der_initial;
    Eigen::VectorXd der_final;
    Eigen::VectorXd der_costs;
    Eigen::VectorXd taus;
    int n_fixed;
    Eigen::MatrixXd intermediate_ders;
    Eigen::VectorXd waypoint_1;
    Eigen::VectorXd waypoint_2;
    double cost;


};


#endif //SPLINES_OPTIMALPIECEWISEPOLYNOMIALGENERATOR_H
