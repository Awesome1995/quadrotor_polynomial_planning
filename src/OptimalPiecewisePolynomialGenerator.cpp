//
// Created by peteflo on 2/5/16.
//

#include "OptimalPiecewisePolynomialGenerator.h"

void OptimalPiecewisePolynomialGenerator::setUpOptimization(){
    n_segments = 3;
    n_derivatives_specified = 5;
    der_initial = Eigen::VectorXd(n_derivatives_specified);
    der_initial << 0, 0, 5, 0, 0;
    der_final = Eigen::VectorXd(n_derivatives_specified);
    der_final << 10, 0, 0, 0, 0;
    der_costs = Eigen::VectorXd(10);
    der_costs << 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
    taus = Eigen::VectorXd(n_segments);
    taus << 0.75, 0.5, 1;
    n_fixed = 1;
    intermediate_ders = Eigen::MatrixXd(n_derivatives_specified, n_segments - 1);

    waypoint_1 = Eigen::VectorXd(n_derivatives_specified);
    waypoint_1 << 4, 0, 0, 0, 0; // Position of .4 for waypoint 1
    intermediate_ders.col(0) << waypoint_1;

    waypoint_2 = Eigen::VectorXd(n_derivatives_specified);
    waypoint_2 << 5, 0, 0, 0, 0; // Position of .5 for waypoint 2
    intermediate_ders.col(1) << waypoint_2;

    Polynomial * polys_unconstrained_sparse[3];
    Eigen::MatrixXd opt_ders_unconstrained; // Optimal derivatives
    Eigen::VectorXd opt_costs; // Return costs for each segment
    GenerateWithFixedTimeSegments(taus, der_initial, der_final, der_costs, intermediate_ders, polys_unconstrained_sparse,
                               opt_ders_unconstrained, opt_costs, n_fixed);

    Polynomial p0_unC_sparse, p1_unC_sparse, p2_unC_sparse;
    p0_unC_sparse = *polys_unconstrained_sparse[0];
    p1_unC_sparse = *polys_unconstrained_sparse[1];
    p2_unC_sparse = *polys_unconstrained_sparse[2];

    std::cout << p0_unC_sparse.eval(0) << std::endl;
    std::cout << p0_unC_sparse.eval(0.75) << std::endl;
    std::cout << p2_unC_sparse.eval(1) << std::endl;

}