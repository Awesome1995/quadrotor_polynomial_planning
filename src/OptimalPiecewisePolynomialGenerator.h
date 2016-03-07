//
// Created by peteflo on 2/5/16.
//

#ifndef SPLINES_OPTIMALPIECEWISEPOLYNOMIALGENERATOR_H
#define SPLINES_OPTIMALPIECEWISEPOLYNOMIALGENERATOR_H

#include "PiecewisePolynomial.h"

struct OptimalPiecewisePolynomial {
    PiecewisePolynomial piecewise_poly;
    Eigen::VectorXd costs;
};


class OptimalPiecewisePolynomialGenerator {
public:

    void setUpOptimizationTest(int n_segments);
    void setUpOptimization(int n_segments);
    void setUpOptimizationWithWaypoints(const Eigen::VectorXd waypoints, const double current_velocity);
    OptimalPiecewisePolynomial GenerateWithFixedTimeSegments(const Eigen::VectorXd & taus);


private:

    void initializeOptimizationCriteria();

    void setInitialPositionConstraint(const double initial_position);
    void setInitialVelocityConstraint(const double initial_velocity);
    void initializeInitialHigherOrderDerivativeConstraints();

    void setFinalPositionConstraint(const double initial_position);
    void setFinalVelocityConstraint(const double initial_velocity);
    void initializeFinalHigherOrderDerivativeConstraints();

    void setPositionWaypoints(Eigen::VectorXd waypoints);
    void setPositionWaypointsTest();
    void setHigherOrderDerivativeWaypoints();

    void GenerateWithFixedTimeSegments(const Eigen::VectorXd & taus, const Eigen::VectorXd & der_0,
                                       const Eigen::VectorXd & der_final, const Eigen::VectorXd & der_costs, const Eigen::MatrixXd & intermediate_der,
                                       Polynomial * polys[], Eigen::MatrixXd & opt_ders, Eigen::VectorXd & costs, int intermediate_ders_fixed = 0);


    int n_segments;
    int n_derivatives_specified;

    double initial_position;
    double initial_velocity;
    Eigen::VectorXd initial_derivatives;

    double final_position;
    double final_velocity;
    Eigen::VectorXd final_derivatives;

    Eigen::VectorXd derivatives_to_minimize;
    int n_fixed;

    Eigen::VectorXd position_waypoints;
    Eigen::MatrixXd intermediate_derivatives;
    Eigen::VectorXd waypoint;

    Eigen::MatrixXd optimal_derivatives; // Optimal derivatives
    Eigen::VectorXd optimal_costs; // Return costs for each segment

    double cost;

};

/**
 * Builds a matrix such that A*p = V, where V(ii) is the ii'th derivative at t=tau and p is a vector of polynomial coefficients
 *
 * The number of derivatives is given by A_derivative.rows(), and the order of the polynomial is A_derivative.cols()-1
 */
void polyGetDerivativeMatrix(double tau, Eigen::MatrixXd & A_derivative, double t_scale = 1.0);

/**
 * Builds a cost matrix such Q, that sum_dd der_costs(dd)*p^(dd)^T*Q*p^(dd) is the integral of p^2(t) from 0 to tau
 */
void polyGetCostMatrix(double tau, Eigen::MatrixXd & Q, const Eigen::VectorXd & der_costs);

/**
 * optimizes a polynomial subject to derivative constraints and t=0 and t=tau and cost on the integral of the squared derivatives
 *
 * the 0th derivative is the polynomial itself and so on
 */
Polynomial polyQuadDerOpt(double tau, const Eigen::VectorXd & der_0, const Eigen::VectorXd & der_final,
                          const Eigen::VectorXd & der_costs, double * cost = NULL);

int sign(int v);

void polyQaudDerOptPiecewiseIndexMap(int N_extra_constraints, int D, int N_poly, int K,
                                     Eigen::VectorXi & index_kk_to_BR);

/**
 * Jointly optimizes a set of polynomials
 *
 * intermediate_der is a matrix with fixed derivatives occupying the first intermediate_ders_fixed rows
 * the rest of the rows are occupied by offset constraints.  Leave intermediate_ders_fixed = 0 to only specify offsets,
 * otherwise set it to the correct number to constrain derivatives.
 *
 */
void polyQuadDerOptPiecewise(const Eigen::VectorXd & taus, const Eigen::VectorXd & der_0,
                             const Eigen::VectorXd & der_final, const Eigen::VectorXd & der_costs, const Eigen::MatrixXd & intermediate_der,
                             Polynomial * polys[], double * cost = NULL, int intermediate_ders_fixed = 0);

/**
 * Jointly optimizes a set of polynomials, solving directly for the waypoint derivatives
 *
 * syntax and use are the same as polyQuadDerOptPiecewise (above), except that this function takes in the matrix argument
 * opt_ders in order to return the matrix of optimal waypoint derivatives in addition to the optimal polynomials
 *
 * enforcing derivative offsets has NOT yet been implemented in this method
 *
 */
void polyOptPiecewiseDers(const Eigen::VectorXd & taus, const Eigen::VectorXd & der_0,
                          const Eigen::VectorXd & der_final, const Eigen::VectorXd & der_costs, const Eigen::MatrixXd & intermediate_der,
                          Polynomial * polys[], Eigen::MatrixXd & opt_ders, double * cost = NULL, int intermediate_ders_fixed = 0);

/*
 * Same as above, only this one uses sparse matrices internally for speed/scalability
 */
void polyOptPiecewiseDersSparse(const Eigen::VectorXd & taus, const Eigen::VectorXd & der_0,
                                const Eigen::VectorXd & der_final, const Eigen::VectorXd & der_costs, const Eigen::MatrixXd & intermediate_der,
                                Polynomial * polys[], Eigen::MatrixXd & opt_ders, Eigen::VectorXd & costs, int intermediate_ders_fixed = 0);

void freeTripletVector(std::vector<Trip *> & ptr_list);


#endif //SPLINES_OPTIMALPIECEWISEPOLYNOMIALGENERATOR_H
