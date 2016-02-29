//
// Created by peteflo on 2/5/16.
//

#include "OptimalPiecewisePolynomialGenerator.h"

void OptimalPiecewisePolynomialGenerator::setUpOptimizationTest(int n_segments){
    this->n_segments = n_segments;
    initializeOptimizationCriteria();

    setInitialPositionConstraint(0.0);
    setInitialVelocityConstraint(0.0);
    initializeInitialHigherOrderDerivativeConstraints();

    setFinalPositionConstraint(10.0);
    setFinalVelocityConstraint(0.0);
    initializeFinalHigherOrderDerivativeConstraints();

    setPositionWaypointsTest();
    setHigherOrderDerivativeWaypoints();
}

void OptimalPiecewisePolynomialGenerator::setUpOptimization(int n_segments){
    this->n_segments = n_segments;
    initializeOptimizationCriteria();
}

void OptimalPiecewisePolynomialGenerator::setUpOptimizationWithWaypoints(const Eigen::VectorXd waypoints_one_output, const double current_velocity){
    // format of waypoints_one_output should be:
    // number of elements = number of waypoitns
    // number of segments = number of elements -1

    this->n_segments = waypoints_one_output.size() - 1;
    initializeOptimizationCriteria();

    setInitialPositionConstraint(waypoints_one_output(0));
    setInitialVelocityConstraint(current_velocity);
    initializeInitialHigherOrderDerivativeConstraints();

    setFinalPositionConstraint(waypoints_one_output(n_segments));
    setFinalVelocityConstraint(0.0);
    initializeFinalHigherOrderDerivativeConstraints();

    setPositionWaypoints(waypoints_one_output.segment(1, n_segments-1));
    setHigherOrderDerivativeWaypoints();

    std::cout << "Initial position " << initial_position << " Initial velocity " << initial_velocity << std::endl;
    std::cout << "Final position " << final_position << " Final velocity " << final_velocity << std::endl;
    std::cout << "Waypoints intermediate " << position_waypoints << std::endl;

};


void OptimalPiecewisePolynomialGenerator::initializeOptimizationCriteria() {
    n_derivatives_specified = 5;

    derivatives_to_minimize = Eigen::VectorXd(10);
    // Choose coefficients for the derivatives to minimize in the optimization
    // Zero-ordered index, so index=3 (fourth in list) is jerk
    //                        index=4 (fifth in list) is snap
    derivatives_to_minimize << 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
}

void OptimalPiecewisePolynomialGenerator::setInitialPositionConstraint(const double initial_position) {
    // In future, this may get from state estimator
    this->initial_position = initial_position;
}

void OptimalPiecewisePolynomialGenerator::setInitialVelocityConstraint(const double initial_velocity) {
    // In future, this may get from state estimator
    this->initial_velocity = initial_velocity;
}

void OptimalPiecewisePolynomialGenerator::initializeInitialHigherOrderDerivativeConstraints() {
    initial_derivatives = Eigen::VectorXd(n_derivatives_specified);
    initial_derivatives << initial_position, initial_velocity, 0, 0, 0;
}



void OptimalPiecewisePolynomialGenerator::setFinalPositionConstraint(const double final_position) {
    // In future, this may get from published waypoints
    this->final_position = final_position;
}

void OptimalPiecewisePolynomialGenerator::setFinalVelocityConstraint(const double final_velocity) {
    // In future, this may get from published waypoints
    this->final_velocity = final_velocity;
}

void OptimalPiecewisePolynomialGenerator::initializeFinalHigherOrderDerivativeConstraints() {
    final_derivatives = Eigen::VectorXd(n_derivatives_specified);
    final_derivatives << final_position, final_velocity, 0, 0, 0;
}


void OptimalPiecewisePolynomialGenerator::setPositionWaypointsTest() {
    position_waypoints = Eigen::VectorXd(n_segments - 1);
    position_waypoints << 4, 5;
}

void OptimalPiecewisePolynomialGenerator::setPositionWaypoints(const Eigen::VectorXd position_waypoints_intermediate){
    position_waypoints = position_waypoints_intermediate;
}

void OptimalPiecewisePolynomialGenerator::setHigherOrderDerivativeWaypoints() {
    n_fixed = 1;
    intermediate_derivatives = Eigen::MatrixXd(n_derivatives_specified, n_segments - 1);
    waypoint = Eigen::VectorXd(n_derivatives_specified);
    for (int i = 0; i < n_segments - 1; i++) {
        waypoint <<  position_waypoints(i), 0, 0, 0, 0; // Position of .4 for waypoint 1
        intermediate_derivatives.col(i) << waypoint;
    }
}

OptimalPiecewisePolynomial OptimalPiecewisePolynomialGenerator::GenerateWithFixedTimeSegments(const Eigen::VectorXd & taus) {

    Polynomial * polys_unconstrained_sparse[n_segments];
    GenerateWithFixedTimeSegments(taus, initial_derivatives, final_derivatives, derivatives_to_minimize, intermediate_derivatives, polys_unconstrained_sparse,
                                  optimal_derivatives, optimal_costs, n_fixed);

    std::vector<std::shared_ptr<Polynomial>> shared_polys;
    for (int i = 0 ; i < n_segments; i++) {
        shared_polys.push_back(std::shared_ptr<Polynomial> (polys_unconstrained_sparse[i]));
    }

    OptimalPiecewisePolynomial optimal_piecewise_poly;
    optimal_piecewise_poly.piecewise_poly = PiecewisePolynomial(shared_polys, taus);
    optimal_piecewise_poly.costs = optimal_costs;

    return optimal_piecewise_poly;

}


void OptimalPiecewisePolynomialGenerator::GenerateWithFixedTimeSegments(const Eigen::VectorXd & taus, const Eigen::VectorXd & der_0,
                                const Eigen::VectorXd & der_final, const Eigen::VectorXd & der_costs, const Eigen::MatrixXd & intermediate_der,
                                Polynomial * polys[], Eigen::MatrixXd & opt_ders, Eigen::VectorXd & costs, int int_ders_fixed)
{
    // Define types needed for sparse matrix operation
    typedef Eigen::SparseMatrix<double> SparseMat;
//  typedef Eigen::Triplet<double> Trip; // Row, column, value

    int N_poly = der_costs.rows();
    int K = taus.rows();
    int D = intermediate_der.rows();
    costs.setZero(K);

    if (N_poly != 2 * D) {
        fprintf(stderr, "Error: Number of costs specified must equal 2x the number of ders specified, %s, line %d\n",
                __FILE__, __LINE__);
        return;
    }

    int N_init_constraints = der_0.rows();
    int N_final_constraints = der_final.rows();
    int N_int_waypts = intermediate_der.cols();


    if (K != 1 + N_int_waypts) {
        fprintf(stderr, "Error: Number of taus must be 1 + number of intermediate waypoints, %s, line %d\n", __FILE__,
                __LINE__);
        return;
    }

    int N_P = N_final_constraints + N_init_constraints + D * N_int_waypts;
    int N_Pd = N_final_constraints + N_init_constraints + 2 * D * N_int_waypts;
    int N_K = K * N_poly;
    int N_Fx = N_final_constraints + N_init_constraints + N_int_waypts * int_ders_fixed;
    int N_F = N_int_waypts * (D - int_ders_fixed);
    int N_R = N_P - N_F;

    Eigen::VectorXd Ders = Eigen::VectorXd::Zero(N_Pd);
    Eigen::MatrixXd Q_kk = Eigen::MatrixXd::Zero(N_poly, N_poly);

    Ders.head(N_init_constraints) = der_0;
    Ders.tail(N_final_constraints) = der_final;
    for (int r = 0; r < N_int_waypts; r++) {
        Ders.segment(N_init_constraints + 2 * r * D, D) = intermediate_der.col(r);
        Ders.segment(N_init_constraints + D * (2 * r + 1), D) = intermediate_der.col(r);
    }

    // Assemble Selector Matrix M
    SparseMat M(N_P, N_Pd);
    std::vector<Trip> M_coeffs;
    std::vector<Trip *> M_coeffs_ptr;

    int m_row = 0;

    int i, j;
    for (i = 0; i < N_init_constraints; i++, m_row++) {
        Trip * c = new Trip(m_row, i, 1);
        M_coeffs.push_back(*c);
        M_coeffs_ptr.push_back(c);
    }

    for (i = 0; i < N_final_constraints; i++, m_row++) {
        Trip * c = new Trip(m_row, N_Pd - N_init_constraints + i, 1);
        M_coeffs.push_back(*c);
        M_coeffs_ptr.push_back(c);
    }

    for (j = 0; j < N_int_waypts; j++) {
        for (i = 0; i < int_ders_fixed; i++, m_row++) {
            Trip * c1 = new Trip(m_row, i + N_init_constraints + 2 * D * j, 1);
            Trip * c2 = new Trip(m_row, i + N_init_constraints + (2 * j + 1) * D, 1);
            M_coeffs.push_back(*c1);
            M_coeffs_ptr.push_back(c1);
            M_coeffs.push_back(*c2);
            M_coeffs_ptr.push_back(c2);
        }
    }

    for (j = 0; j < N_int_waypts; j++) {
        for (i = int_ders_fixed; i < D; i++, m_row++) {
            Trip * c1 = new Trip(m_row, i + N_init_constraints + 2 * D * j, 1);
            Trip * c2 = new Trip(m_row, i + N_init_constraints + (2 * j + 1) * D, 1);
            M_coeffs.push_back(*c1);
            M_coeffs_ptr.push_back(c1);
            M_coeffs.push_back(*c2);
            M_coeffs_ptr.push_back(c2);
        }
    }
    M.setFromTriplets(M_coeffs.begin(), M_coeffs.end());
    freeTripletVector(M_coeffs_ptr);
//  eigen_dump(Eigen::MatrixXd::Identity(N_P,N_P)*M);

    // Assemble Cost Matrix Q
    SparseMat Q(N_K, N_K);
    std::vector<Trip> Q_coeffs;
    std::vector<Trip *> Q_coeffs_ptr;

    for (int kk = 0; kk < K; kk++) {
        polyGetCostMatrix(taus(kk), Q_kk, der_costs);
        for (i = 0; i < N_poly; i++) {
            for (j = 0; j < N_poly; j++) {
                Trip * q = new Trip(N_poly * kk + i, N_poly * kk + j, Q_kk.coeff(i, j));
                Q_coeffs.push_back(*q);
                Q_coeffs_ptr.push_back(q);
            }
        }
    }
    Q.setFromTriplets(Q_coeffs.begin(), Q_coeffs.end());
    freeTripletVector(Q_coeffs_ptr);
//  eigen_dump(Eigen::MatrixXd::Identity(N_K,N_K)*Q);

    // Assemble Constraint Matrix A
    SparseMat A_inv(N_Pd, N_Pd);
    std::vector<Trip> A_inv_coeffs;
    std::vector<Trip *> A_inv_coeffs_ptr;
    A_inv_coeffs.clear();

    Eigen::MatrixXd A_final_cc = Eigen::MatrixXd::Zero(D, N_poly);
    Eigen::MatrixXd A_init_cc = Eigen::MatrixXd::Zero(D, N_poly);
    Eigen::MatrixXd A_cc = Eigen::MatrixXd::Zero(N_poly, N_poly);
    Eigen::MatrixXd A_cc_inv = Eigen::MatrixXd::Zero(N_poly, N_poly);
    for (int cc = 0; cc <= N_int_waypts; cc++) {
        polyGetDerivativeMatrix(0, A_init_cc);
        polyGetDerivativeMatrix(taus(cc), A_final_cc);
        A_cc.topRows(D) = A_init_cc;
        A_cc.bottomRows(D) = A_final_cc;
        A_cc_inv = A_cc.inverse();
        for (i = 0; i < D; i++) {
            for (j = 0; j < N_poly; j++) {
                Trip * a1 = new Trip(2 * cc * D + i, cc * N_poly + j, A_cc_inv.coeff(i, j));
                Trip * a2 = new Trip(2 * cc * D + D + i, cc * N_poly + j, A_cc_inv.coeff(D + i, j));
                A_inv_coeffs.push_back(*a1);
                A_inv_coeffs_ptr.push_back(a1);
                A_inv_coeffs.push_back(*a2);
                A_inv_coeffs_ptr.push_back(a2);
            }
        }
    }
    A_inv.setFromTriplets(A_inv_coeffs.begin(), A_inv_coeffs.end());
    freeTripletVector(A_inv_coeffs_ptr);
//  eigen_dump(Eigen::MatrixXd::Identity(N_Pd,N_Pd)*A_inv);

    SparseMat R = M * A_inv.transpose() * Q * A_inv * M.transpose();
//  eigen_dump(Eigen::MatrixXd::Identity(N_P,N_P)*R);

    SparseMat Rpp(N_F, N_F);
    Eigen::MatrixXd Rfp_transpose = Eigen::MatrixXd::Zero(N_F, N_R);
    std::vector<Trip> Rpp_coeffs;
    std::vector<Trip *> Rpp_coeffs_ptr;
    Rpp_coeffs.clear();

    for (int k = 0; k < R.outerSize(); ++k)
        for (SparseMat::InnerIterator it(R, k); it; ++it) {

            if (it.row() >= N_P - N_F && it.col() >= N_P - N_F) {
                Trip * rpp = new Trip(it.row() - N_R, it.col() - N_R, it.value());
                Rpp_coeffs.push_back(*rpp);
                Rpp_coeffs_ptr.push_back(rpp);
            }

            else if (it.row() < N_P - N_F && it.col() >= N_P - N_F) {
                Rfp_transpose(it.col() - N_R, it.row()) = it.value();
            }
        }
    Rpp.setFromTriplets(Rpp_coeffs.begin(), Rpp_coeffs.end());
    freeTripletVector(Rpp_coeffs_ptr);

    Eigen::VectorXd ders_fixed = Eigen::VectorXd::Zero(N_Fx);
    ders_fixed.head(N_init_constraints) = der_0;
    ders_fixed.segment(N_init_constraints, N_final_constraints) = der_final;
    for (int jj = 0; jj < N_int_waypts; jj++) {
        ders_fixed.segment(N_init_constraints + N_final_constraints + int_ders_fixed * jj, int_ders_fixed) =
                intermediate_der.block(0, jj, int_ders_fixed, 1);
    }

    Eigen::SimplicialCholesky<SparseMat> Rpp_decomp(Rpp);
    Eigen::VectorXd D_star = -Rpp_decomp.solve(Rfp_transpose) * ders_fixed;

    opt_ders = Eigen::MatrixXd::Zero(D, 2 + N_int_waypts);
    opt_ders.leftCols(1) = der_0;
    opt_ders.rightCols(1) = der_final;
    opt_ders.block(0, 1, int_ders_fixed, N_int_waypts) = intermediate_der.topRows(int_ders_fixed);

    for (int ii = 0; ii < N_int_waypts; ii++) {
        opt_ders.block(int_ders_fixed, 1 + ii, D - int_ders_fixed, 1) = D_star.segment(ii * (D - int_ders_fixed),
                                                                                       D - int_ders_fixed);
    }

    double cost_individual;
    double cost_total = 0;
    for (int kk = 0; kk < K; kk++) {
        polys[kk] = new Polynomial();
        *polys[kk] = polyQuadDerOpt(taus(kk), opt_ders.col(kk), opt_ders.col(kk + 1), der_costs, &cost_individual);
        cost_total += cost_individual;
        costs(kk) = cost_individual;
    }
//  *cost = cost_total;



}


void polyQaudDerOptPiecewiseIndexMap(int N_extra_constraints, int D, int N_poly, int K,
                                     Eigen::VectorXi & index_kk_to_BR)
{

    int N_K = K * N_poly;
    int N_extra_per_K = ((double) N_extra_constraints) / ((double) K); //integer roundoff towards 0 intentionally

    int N_extra_remainder = N_extra_constraints - N_extra_per_K * K;

    int sign_remainder = sign(N_extra_remainder);
    N_extra_remainder = abs(N_extra_remainder);

    Eigen::VectorXi b_kk_sizes = Eigen::VectorXi::Constant(K, D + N_extra_per_K);
    b_kk_sizes.topRows(N_extra_remainder) = b_kk_sizes.segment(0, N_extra_remainder)
                                            + Eigen::VectorXi::Constant(N_extra_remainder, sign_remainder);

    Eigen::VectorXi r_kk_sizes = -b_kk_sizes + Eigen::VectorXi::Constant(K, N_poly);

    int B_start = 0;
    int B_size = b_kk_sizes.sum();
    int R_start = B_size;
    int R_size = r_kk_sizes.sum();

    index_kk_to_BR.setZero(N_K);

    int b_kk_start = 0;
    int r_kk_start = 0;

    Eigen::VectorXi x_kk_inds;
    for (int kk = 0; kk < K; kk++) {
        x_kk_inds.setLinSpaced(N_poly, kk * N_poly, (kk + 1) * N_poly - 1);
        index_kk_to_BR.segment(B_start, B_size).segment(b_kk_start, b_kk_sizes(kk)) = x_kk_inds.topRows(b_kk_sizes(kk));
        index_kk_to_BR.segment(R_start, R_size).segment(r_kk_start, r_kk_sizes(kk)) = x_kk_inds.bottomRows(r_kk_sizes(kk));
        b_kk_start += b_kk_sizes(kk);
        r_kk_start += r_kk_sizes(kk);
    }
}


void polyQuadDerOptPiecewise(const Eigen::VectorXd & taus, const Eigen::VectorXd & der_0,
                             const Eigen::VectorXd & der_final, const Eigen::VectorXd & der_costs, const Eigen::MatrixXd & intermediate_der,
                             Polynomial * polys[], double * cost, int intermediate_ders_fixed)
{
    int N_poly = der_costs.rows();
    int K = taus.rows();
    int D = intermediate_der.rows();

    int N_init_constraints = der_0.rows();
    int N_final_constraints = der_final.rows();
    int N_B;
    N_B = N_final_constraints + N_init_constraints + (K - 1) * (D - intermediate_ders_fixed)
          + 2 * (K - 1) * intermediate_ders_fixed; //total number of constraints
    int N_K = K * N_poly; //number of free parameters
    int N_R = N_K - N_B; //number of parameters to optimize

    if (N_R < 0) {
        fprintf(stderr,
                "Error: Not enough free parameters (polynomial degree must by higher) in polyQuadDerOptPiecewise, %s, line %d\n",
                __FILE__, __LINE__);

        for (int kk = 0; kk < K; kk++) {
            polys[kk]->coeffs.setZero(N_poly + 1);
        }
        return;
    }

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(N_B, N_K);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(N_B);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(N_K, N_K);
    Eigen::MatrixXd Q_kk = Eigen::MatrixXd::Zero(N_poly, N_poly);

    for (int kk = 0; kk < K; kk++) {
        polyGetCostMatrix(taus(kk), Q_kk, der_costs);
        int kk_start = kk * N_poly;
        Q.block(kk_start, kk_start, N_poly, N_poly) = Q_kk;
    }

    int constraint_start_ind = 0;
    Eigen::MatrixXd A_final_cc = Eigen::MatrixXd::Zero(D, N_poly);
    Eigen::MatrixXd A_init_cc = Eigen::MatrixXd::Zero(D, N_poly);
    for (int cc = 0; cc <= K; cc++) { //K+1 constraint blocks
        int kk_final = cc - 1;
        int kk_init = cc;
        int final_block_ind = kk_final * N_poly;
        int init_block_ind = kk_init * N_poly;
        if (cc == 0) {
            Eigen::MatrixXd A_init_0 = Eigen::MatrixXd::Zero(N_init_constraints, N_poly);
            polyGetDerivativeMatrix(0, A_init_0);
            A.block(constraint_start_ind, init_block_ind, N_init_constraints, N_poly) = A_init_0;
            b.segment(constraint_start_ind, N_init_constraints) = der_0;
            constraint_start_ind += N_init_constraints;
        }
        else if (cc < K) {
            polyGetDerivativeMatrix(taus(kk_final), A_final_cc);
            polyGetDerivativeMatrix(0, A_init_cc);

            // First constrain the intermediate derivatives
            if (intermediate_ders_fixed != 0) {
                A.block(constraint_start_ind, final_block_ind, intermediate_ders_fixed, N_poly) = A_final_cc.topRows(
                        intermediate_ders_fixed);
                b.segment(constraint_start_ind, intermediate_ders_fixed) = intermediate_der.col(cc - 1).head(
                        intermediate_ders_fixed);
                constraint_start_ind += intermediate_ders_fixed;
                A.block(constraint_start_ind, init_block_ind, intermediate_ders_fixed, N_poly) = A_init_cc.topRows(
                        intermediate_ders_fixed);
                b.segment(constraint_start_ind, intermediate_ders_fixed) = intermediate_der.col(cc - 1).head(
                        intermediate_ders_fixed);
                constraint_start_ind += intermediate_ders_fixed;
            }

            int num_offset_constraints = D - intermediate_ders_fixed;
            A.block(constraint_start_ind, final_block_ind, num_offset_constraints, N_poly) = -A_final_cc.bottomRows(
                    num_offset_constraints);
            A.block(constraint_start_ind, init_block_ind, num_offset_constraints, N_poly) = A_init_cc.bottomRows(
                    num_offset_constraints);
            b.segment(constraint_start_ind, num_offset_constraints) = intermediate_der.col(cc - 1).tail(
                    num_offset_constraints);
            constraint_start_ind += num_offset_constraints;

        }
        else {
            Eigen::MatrixXd A_final_K = Eigen::MatrixXd::Zero(N_final_constraints, N_poly);
            polyGetDerivativeMatrix(taus(kk_final), A_final_K);
            A.block(constraint_start_ind, final_block_ind, N_final_constraints, N_poly) = -A_final_K;
            b.segment(constraint_start_ind, N_final_constraints) = -der_final;
            constraint_start_ind += N_final_constraints;
        }
    }

    int N_extra_constraints = N_init_constraints + N_final_constraints - D;

    Eigen::VectorXi index_kk_to_BR;

    polyQaudDerOptPiecewiseIndexMap(N_extra_constraints, D, N_poly, K, index_kk_to_BR);

    Eigen::MatrixXd A_BR = Eigen::MatrixXd::Zero(N_B, N_K);
    Eigen::MatrixXd Q_BR = Eigen::MatrixXd::Zero(N_K, N_K);

    Eigen::VectorXi test_indexing(N_K);

    for (int i = 0; i < N_K; i++) {
        A_BR.col(i) = A.col(index_kk_to_BR(i));
        for (int j = 0; j < N_K; j++) {
            Q_BR(i, j) = Q(index_kk_to_BR(i), index_kk_to_BR(j));
        }
    }

    Eigen::VectorXd x_star_BR(N_K);
    Eigen::VectorXd x_star(N_K);
    eigen_utils::quadProgEliminationSolve(Q_BR, A_BR, b, x_star_BR);

    for (int i = 0; i < N_K; i++) {
        x_star(index_kk_to_BR(i)) = x_star_BR(i);
    }

    for (int kk = 0; kk < K; kk++) {

        if (taus(kk) <= .0000001) {
            x_star.segment(kk * N_poly, N_poly).setZero(); //if we have 0 length segment, 0 the polynomial
            fprintf(stderr,
                    "error, 0 length segment in polyQuadDerOptPiecewise, setting coefficients to 0, may cause other problems\n");
        }
        polys[kk]->coeffs = x_star.segment(kk * N_poly, N_poly);

//    eigen_matlab_dump(*polys[kk]);
    }

    if (cost != NULL) {
        *cost = x_star.transpose() * Q * x_star;
    }
//  eigen_matlab_dump(Q);
//  eigen_matlab_dump(A);
//  eigen_matlab_dump(b);
//  eigen_matlab_dump(index_kk_to_BR);
//  eigen_matlab_dump(A_BR);
//  eigen_matlab_dump(Q_BR);
//  eigen_matlab_dump(x_star);
//  eigen_matlab_dump(x_star_BR);
}

void polyOptPiecewiseDers(const Eigen::VectorXd & taus, const Eigen::VectorXd & der_0,
                          const Eigen::VectorXd & der_final, const Eigen::VectorXd & der_costs, const Eigen::MatrixXd & intermediate_der,
                          Polynomial * polys[], Eigen::MatrixXd & opt_ders, double * cost, int intermediate_ders_fixed)
{
    /*
     * Assumes that the same number of ders are given for start, end and intermediate points.
     * Requires that the number of costs be appropriate for the minimum order polynomial
     *    i.e. if der_0 has dimension 4 and all intermediate_der vectors are dimension 4, then der_costs
     *    should have dimension 8
     */

    int N_poly = der_costs.rows();
    int K = taus.rows();
    int D = intermediate_der.rows();

    if (N_poly != 2 * D) {
        fprintf(stderr, "Error: Number of costs specified must equal 2x the number of ders specified, %s, line %d\n",
                __FILE__, __LINE__);
        return;
    }

    int N_init_constraints = der_0.rows();
    int N_final_constraints = der_final.rows();
    int N_int_waypts = intermediate_der.cols();
    int N_P = N_final_constraints + N_init_constraints + D * N_int_waypts;
    int N_Pd = N_final_constraints + N_init_constraints + 2 * D * N_int_waypts;
    int N_K = K * N_poly;
    int N_Fx = N_final_constraints + N_init_constraints + N_int_waypts * intermediate_ders_fixed;
    int N_F = N_int_waypts * (D - intermediate_ders_fixed);
    int N_R = N_P - N_F;

    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(N_P, N_Pd);
    Eigen::MatrixXd M_int_rows = Eigen::MatrixXd::Zero(D * N_int_waypts, N_Pd);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(N_Pd, N_Pd);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(N_K, N_K);
    Eigen::MatrixXd Q_kk = Eigen::MatrixXd::Zero(N_poly, N_poly);
    Eigen::VectorXd Ders = Eigen::VectorXd::Zero(N_Pd);

    Ders.head(N_init_constraints) = der_0;
    Ders.tail(N_final_constraints) = der_final;
    for (int r = 0; r < N_int_waypts; r++) {
        Ders.segment(N_init_constraints + 2 * r * D, D) = intermediate_der.col(r);
        Ders.segment(N_init_constraints + D * (2 * r + 1), D) = intermediate_der.col(r);
    }

    M.topLeftCorner(D, D).setIdentity(); // Initial constraint
    M.block(N_init_constraints, N_Pd - N_init_constraints, D, D).setIdentity(); // Final constraint

    // Assemble the rows of M representing the intermediate derivatives
    for (int i = 0; i < N_int_waypts; i++) {
        M_int_rows.block(i * D, N_init_constraints + i * 2 * D, D, D).setIdentity();
        M_int_rows.block(i * D, N_init_constraints + i * 2 * D + D, D, D).setIdentity();
    }

    // Insert the M rows corresponding to int ders that are fixed
    int cur_m_row = N_init_constraints + N_final_constraints;
    for (int i = 0; i < N_int_waypts; i++) {
        for (int j = 0; j < intermediate_ders_fixed; j++) {
            M.row(cur_m_row) = M_int_rows.row(i * D + j);
            cur_m_row++;
        }
    }

    // Insert the M rows corresponding to int ders that are free
    for (int ii = 0; ii < N_int_waypts; ii++) {
        for (int jj = intermediate_ders_fixed; jj < D; jj++) {
            M.row(cur_m_row) = M_int_rows.row(ii * D + jj);
            cur_m_row++;
        }
    }
//  eigen_dump(M);

    for (int kk = 0; kk < K; kk++) {
        polyGetCostMatrix(taus(kk), Q_kk, der_costs);
        int kk_start = kk * N_poly;
        Q.block(kk_start, kk_start, N_poly, N_poly) = Q_kk;
    }
//  eigen_dump(Q);

    Eigen::MatrixXd A_final_cc = Eigen::MatrixXd::Zero(D, N_poly);
    Eigen::MatrixXd A_init_cc = Eigen::MatrixXd::Zero(D, N_poly);
    for (int cc = 0; cc <= N_int_waypts; cc++) {
        polyGetDerivativeMatrix(0, A_init_cc);
        polyGetDerivativeMatrix(taus(cc), A_final_cc);
        A.block(2 * cc * D, cc * N_poly, D, N_poly) = A_init_cc;
        A.block(2 * cc * D + D, cc * N_poly, D, N_poly) = A_final_cc;
    }
//  eigen_dump(A);

    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> A_trans_decomp;
    A_trans_decomp.compute(A.transpose());
//  if (!A_trans_decomp.isInvertible()) {
//    fprintf(stderr,
//        "Warning: A_transpose matrix for derivative optimization in %s is not invertible using ColPivHouseholderQR, line %d\n",
//        __FILE__, __LINE__);
//  }

    Eigen::MatrixXd A_trans_inv_Q_trans = A_trans_decomp.solve(Q.transpose());
    Eigen::MatrixXd Q_total = M * A_trans_decomp.solve(A_trans_inv_Q_trans.transpose()) * M.transpose();
    Eigen::MatrixXd Qpp, Qfp;
    Qpp = Q_total.bottomRightCorner(N_F, N_F);
    Qfp = Q_total.topRightCorner(N_R, N_F);

//  eigen_dump(A_trans_inv_Q_trans);
//  eigen_dump(Q_total);

    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> Qpp_decomp;
    Qpp_decomp.compute(Qpp);
//  if (!Qpp_decomp.isInvertible()) {
//    fprintf(stderr,
//        "Warning: Qpp matrix for derivative optimization in %s is not invertible using ColPivHouseholderQR, line %d\n",
//        __FILE__, __LINE__);
//  }

    Eigen::VectorXd ders_fixed = Eigen::VectorXd::Zero(N_Fx);
    ders_fixed.head(N_init_constraints) = der_0;
    ders_fixed.segment(N_init_constraints, N_final_constraints) = der_final;
    for (int jj = 0; jj < N_int_waypts; jj++) {
        ders_fixed.segment(N_init_constraints + N_final_constraints + intermediate_ders_fixed * jj, intermediate_ders_fixed) =
                intermediate_der.block(0, jj, intermediate_ders_fixed, 1);
    }
    Eigen::VectorXd solution = -Qpp_decomp.solve(Qfp.transpose()) * ders_fixed;

    opt_ders = Eigen::MatrixXd::Zero(D, 2 + N_int_waypts);
    opt_ders.leftCols(1) = der_0;
    opt_ders.rightCols(1) = der_final;
    opt_ders.block(0, 1, intermediate_ders_fixed, N_int_waypts) = intermediate_der.topRows(intermediate_ders_fixed);

    for (int ii = 0; ii < N_int_waypts; ii++) {
        opt_ders.block(intermediate_ders_fixed, 1 + ii, D - intermediate_ders_fixed, 1) = solution.segment(
                ii * (D - intermediate_ders_fixed), D - intermediate_ders_fixed);
    }

    double cost_individual;
    double cost_total = 0;
    for (int kk = 0; kk < K; kk++) {
        polys[kk] = new Polynomial();
        *polys[kk] = polyQuadDerOpt(taus(kk), opt_ders.col(kk), opt_ders.col(kk + 1), der_costs, &cost_individual);
        cost_total += cost_individual;
    }
    *cost = cost_total;
}


Polynomial polyQuadDerOpt(double tau, const Eigen::VectorXd & der_0, const Eigen::VectorXd & der_final,
                          const Eigen::VectorXd & der_costs, double * cost)
{

    int N_poly = der_costs.rows();
    int N_init_constraints = der_0.rows();
    int N_final_constraints = der_final.rows();
    int N_B = N_final_constraints + N_init_constraints;
    int N_R = N_poly - N_B;

    if (N_R < 0) {
        fprintf(stderr,
                "Error: Not enough free parameters (polynomial degree must by higher) in polyQuadDerOpt, %s, line %d\n",
                __FILE__, __LINE__);
        return Polynomial(N_poly - 1);
    }

    if (tau < .00000001) {
        fprintf(stderr, "error, 0 length segment in polyQuadDerOpt, setting coefficients to 0, may cause other problems\n");
        return Polynomial(N_poly - 1);
    }

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(N_B, N_poly);
    Eigen::MatrixXd A_0(N_init_constraints, N_poly);
    Eigen::MatrixXd A_final(N_final_constraints, N_poly);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(N_B);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(N_poly, N_poly);

    polyGetDerivativeMatrix(0, A_0);
    polyGetDerivativeMatrix(tau, A_final);
    A.topRows(N_init_constraints) = A_0;
    A.bottomRows(N_final_constraints) = A_final;
    b.topRows(N_init_constraints) = der_0;
    b.bottomRows(N_final_constraints) = der_final;

    polyGetCostMatrix(tau, Q, der_costs);

    Polynomial poly(N_poly - 1);

    eigen_utils::quadProgEliminationSolve(Q, A, b, poly.coeffs);

    if (cost != NULL) {
        *cost = poly.coeffs.transpose() * Q * poly.coeffs;
    }

    return poly;
}

void polyGetDerivativeMatrix(double tau, Eigen::MatrixXd & A_derivative, double t_scale)
{
    int N_poly = A_derivative.cols();
    int D = A_derivative.rows();

    A_derivative.setZero();

    for (int r = 0; r < D; r++) {
        for (int n = 0; n < N_poly; n++) {
            if (n >= r) {
                double prod = 1.0;
                for (int m = 0; m < r; m++) {
                    prod = prod * ((double) (n - m)) / t_scale;
                }
                A_derivative(r, n) = prod * pow(tau, n - r);
            }
        }
    }
}

/**
 * Builds a cost matrix such Q, that sum_dd der_costs(dd)*p^(dd)^T*Q*p^(dd) is the integral of p^2(t) from 0 to tau
 */
void polyGetCostMatrix(double tau, Eigen::MatrixXd & Q, const Eigen::VectorXd & der_costs)
{
    assert(der_costs.rows()==Q.rows());
    assert(der_costs.cols()==1);
    assert(Q.cols()==Q.rows());

    int N_poly = Q.cols();
    Q.setZero();

    for (int l = 0; l < N_poly; l++) {
        for (int i = 0; i < N_poly; i++) {
            for (int r = 0; r < N_poly; r++) {
                if (i >= r && l >= r) {
                    double prod = 1.0;
                    for (int m = 0; m < r; m++) {
                        prod *= (i - m) * (l - m);
                    }
                    Q(i, l) += pow(der_costs(r), 2) * 2 * prod * pow(tau, l + i + 1 - 2 * r) / ((double) (l + i + 1 - 2 * r));
                }
            }
        }
    }
}

void freeTripletVector(std::vector<Trip *> & trip_list)
{
    std::vector<Trip *>::iterator ptr_it = trip_list.begin();
    while (!trip_list.empty()) {
        Trip * cur_ptr = *ptr_it;
        ptr_it = trip_list.erase(ptr_it);
        delete cur_ptr;
    }
}