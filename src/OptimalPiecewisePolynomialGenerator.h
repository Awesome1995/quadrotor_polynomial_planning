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

    void setOptimizationCriteria();

    void setInitialPositionConstraint(const double initial_position);
    void setInitialVelocityConstraint(const double initial_velocity);
    void setInitialHigherOrderDerivativeConstraints();

    void setFinalPositionConstraint(const double initial_position);
    void setFinalVelocityConstraint(const double initial_velocity);
    void setFinalHigherOrderDerivativeConstraints();

    void setPositionWaypoints();
    void setHigherOrderDerivativeWaypoints();


private:
    int n_segments;
    int n_derivatives_specified;

    double initial_position;
    double initial_velocity;
    Eigen::VectorXd initial_derivatives;

    double final_position;
    double final_velocity;
    Eigen::VectorXd final_derivatives;

    Eigen::VectorXd derivatives_to_minimize;
    Eigen::VectorXd taus;
    int n_fixed;

    Eigen::MatrixXd intermediate_derivatives;
    Eigen::VectorXd waypoint_1;
    Eigen::VectorXd waypoint_2;
    double cost;


};


#endif //SPLINES_OPTIMALPIECEWISEPOLYNOMIALGENERATOR_H
