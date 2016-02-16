//
// Created by peteflo on 2/5/16.
//

#include "TimeOptimizedQuadSplineGenerator.h"

OptimalPiecewisePolynomial TimeOptimizedQuadSplineGenerator::GenerateTimeOptimized() {
    k_T = 10000.0;  // k_T is the proportional cost on time, as explained in Richter et al 2013, ISRR

    // Set up optimization for optimal piecewise polynomial generator
    n_segments = 3;
    optimal_piecewise_polynomial_generator.setUpOptimization(n_segments);

    // With initial tau guess, generate optimal piecewise polynomial
    Eigen::VectorXd initial_taus = Eigen::VectorXd(n_segments);
    initial_taus << 0.75, 0.5, 1;
    OptimalPiecewisePolynomial initial_optimal_piecewise_poly = optimal_piecewise_polynomial_generator.GenerateWithFixedTimeSegments(initial_taus);
    std::cout << "INITIAL TAUS " << initial_taus << std::endl;
    std::cout << "COSTS " << initial_optimal_piecewise_poly.costs << std::endl;
    std::cout << "SUM OF COSTS " << initial_optimal_piecewise_poly.costs.sum() << std::endl;
    OptimalPiecewisePolynomial current_optimal_piecewise_poly;
    Eigen::VectorXd current_taus = initial_taus;

    for (int i = 0; i < 1000; i++) {
        // Find numerical gradient
        Eigen::VectorXd current_gradient = numericalGradient(current_taus);

        // Take gradient descent step
        Eigen::VectorXd next_taus = oneStepGradientDescent(current_taus, current_gradient);

        current_optimal_piecewise_poly = optimal_piecewise_polynomial_generator.GenerateWithFixedTimeSegments(next_taus);

        current_taus = next_taus;
    }

    std::cout << "FINAL TAUS " << current_taus << std::endl;
    std::cout << "COSTS " << current_optimal_piecewise_poly.costs << std::endl;
    std::cout << "SUM OF COSTS " << current_optimal_piecewise_poly.costs.sum() << std::endl;

    return current_optimal_piecewise_poly;

}


Eigen::VectorXd TimeOptimizedQuadSplineGenerator::numericalGradient(Eigen::VectorXd current_taus) {
    double dx = 0.00001;
    Eigen::VectorXd grad = Eigen::VectorXd(n_segments);

    // Be careful they are not referring to same thing
    Eigen::VectorXd taus_plus = current_taus * 1.0;
    Eigen::VectorXd taus_minus = current_taus * 1.0;

    for (int i = 0; i < n_segments; i++) {
        taus_plus(i) = taus_plus(i) + 1/2.0*dx;
        taus_minus(i) = taus_minus(i) - 1/2.0*dx;

        OptimalPiecewisePolynomial plus_optimal_piecewise_poly = optimal_piecewise_polynomial_generator.GenerateWithFixedTimeSegments(taus_plus);
        double cost_plus = plus_optimal_piecewise_poly.costs.sum() + k_T*taus_plus.sum();

        OptimalPiecewisePolynomial minus_optimal_piecewise_poly = optimal_piecewise_polynomial_generator.GenerateWithFixedTimeSegments(taus_minus);
        double cost_minus = minus_optimal_piecewise_poly.costs.sum() + k_T*taus_minus.sum();

        grad(i) = 1.0 / dx * (cost_plus - cost_minus);

        // reset taus_plus and taus_minus
        taus_plus = current_taus * 1.0;
        taus_minus = current_taus * 1.0;
    }

    //std::cout << "GRAD " << grad << std::endl;

    return grad;
}

Eigen::VectorXd TimeOptimizedQuadSplineGenerator::oneStepGradientDescent(Eigen::VectorXd current_taus, Eigen::VectorXd current_gradient) {
    double stepSize = 0.00000001;
    Eigen::VectorXd next_taus;
    next_taus = current_taus - stepSize * current_gradient;

    //std::cout << "CURRENT TAUS " << current_taus << std::endl;
    //std::cout << "NEXT TAUS " << next_taus << std::endl;

    return next_taus;

}

