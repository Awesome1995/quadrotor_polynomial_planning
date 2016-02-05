#include <iostream>
//#include <eigen_utils/eigen_utils.hpp>
#include <stdlib.h>
#include <time.h>
#include "Polynomial.hpp"
#include <stdio.h>


using namespace std;
using namespace Eigen;
//using namespace eigen_utils;

void poly_test(int argc, char ** argv)
{
  if (argc < 2) {
    printf("usage: %s N\n", argv[0]);
    exit(0);
  }

  int N = atoi(argv[1]);
  int N_poly = N + 1;
  Polynomial poly(N);
  poly.coeffs.setRandom();

  eigen_dump(poly);
  eigen_dump(poly.getDerivative());
  eigen_dump(poly.eval(2));
  eigen_dump(poly.eval(2, 1));
  eigen_dump(poly.getDerivative().eval(2));

  VectorXd coeffs(6);
  coeffs << 1, 1, 1, 1, 1, 1;
  Polynomial poly_coeffs(coeffs);
  eigen_dump(poly_coeffs);

  MatrixXd A_derivative(N, N_poly);
  polyGetDerivativeMatrix(2, A_derivative, 1);
  eigen_dump(A_derivative);

  MatrixXd Q(N_poly, N_poly);
  VectorXd der_costs(N_poly);
  der_costs.setOnes();
  polyGetCostMatrix(2, Q, der_costs);
  eigen_dump(Q);
}

void quad_trajectory_poly_test()
{
  /*
   * Solve for the sequence of polynomial splines with specified initial and final derivatives,
   * and a certain number of specified intermediate derivatives
   */
  int n_segments = 3;
  int n_derivatives_specified = 5;

  // Here is our set of initial derivatives: der_initial = [position_0, velocity_0, accel_0, jerk_0, snap_0]
  // Often, we want our vehicle to start from rest, so we set these derivatives to be all zeros, although
  // you can set whatever initial state you want. These values become constraints in the optimization.
  VectorXd der_initial(n_derivatives_specified);
  der_initial << 0, 0, 5, 0, 0;

  // Here is our set of final derivatives: der_final = [position_F, velocity_F, accel_F, jerk_F, snap_F]
  // Let's say we want to end up at position_F = 1, so we set that element to 1, and if we want to end at rest,
  // then we set all the other final derivatives to zero.
  VectorXd der_final(n_derivatives_specified);
  der_final << 10, 0, 0, 0, 0;

  // Now we specify the penalty/cost to apply to each of the derivatives.  If we set the cost coefficients as follows,
  // we will be minimizing snap (4th derivative, so 5th in list), or jerk (3rd derivative, so 4th in list)
  VectorXd der_costs(10);
  der_costs << 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;

  // Let's say we want a sequence of 3 splines. We need to tell the optimizer how much time to spend on each segment.
  VectorXd taus(n_segments);
  taus << 0.75, 0.5, 1; // (spend .75 seconds on the first segment, .5 seconds on the second segment...)

  // And let's say that we want to go through a few intermediate states along the way.

  // If we want to specify positions of intermediate waypoints, set the number of fixed intermediate derivatives
  // to 1.  If we want to specify position *and velocity* of intermediate waypoints, we would set n_fixed = 2, etc.
  int n_fixed = 1;

  // Now we need to say what those intermediate positions actually are. Since n_derivatives_specified = 5,
  // We will need to specify the first 5 derivatives of each waypoint (pos, vel, accel, jerk, snap). But,
  // since n_fixed = 1, only the first value (position) will be used. We can just put in zeros for the rest,
  // and those zeros will be ignored.
  MatrixXd intermediate_ders(n_derivatives_specified, n_segments - 1);

  // First waypoint
  VectorXd waypoint_1(n_derivatives_specified);
  waypoint_1 << 4, 0, 0, 0, 0; // Position of .4 for waypoint 1
  intermediate_ders.col(0) << waypoint_1;

  // Second waypoint
  VectorXd waypoint_2(n_derivatives_specified);
  waypoint_2 << 5, 0, 0, 0, 0; // Position of .5 for waypoint 2
  intermediate_ders.col(1) << waypoint_2;

  // Return cost
  double cost;

  // Compute the solution using the constrained QP formulation (Mellinger)
  // For this one, we need to allocate Polynomial objects outisde the solver
  Polynomial p0, p1, p2;
  Polynomial * polys_constrained[3];
  polys_constrained[0] = &p0;
  polys_constrained[1] = &p1;
  polys_constrained[2] = &p2;
  polyQuadDerOptPiecewise(taus, der_initial, der_final, der_costs, intermediate_ders, polys_constrained, &cost,
      n_fixed);

  // Print the resulting coefficients to the console
  eigen_matlab_dump(taus);
  eigen_matlab_dump(p0);
  eigen_matlab_dump(p1);
  eigen_matlab_dump(p2);


  // Compute the solution using the unconstrained QP formulation (Richter, Bry & Roy, ISRR 2013)
  // For this one, the solver will allocate the Polynomial object for us
  Polynomial * polys_unconstrained[3];
  Eigen::MatrixXd opt_ders; // Optimal derivatives
  polyOptPiecewiseDers(taus, der_initial, der_final, der_costs, intermediate_ders, polys_unconstrained, opt_ders, &cost,
      n_fixed);

  // Print the resulting coefficients to the console
  Polynomial p0_unC, p1_unC, p2_unC;
  p0_unC = *polys_unconstrained[0];
  p1_unC = *polys_unconstrained[1];
  p2_unC = *polys_unconstrained[2];
  eigen_matlab_dump(p0_unC);
  eigen_matlab_dump(p1_unC);
  eigen_matlab_dump(p2_unC);

  // Compute the solution using the SPARSE implementation of the unconstrained QP (Richter, Bry & Roy, ISRR 2013)
  // For this one, the solver will allocate the Polynomial object for us
  Polynomial * polys_unconstrained_sparse[3];
  Eigen::MatrixXd opt_ders_unconstrained; // Optimal derivatives
  Eigen::VectorXd opt_costs; // Return costs for each segment
  polyOptPiecewiseDersSparse(taus, der_initial, der_final, der_costs, intermediate_ders, polys_unconstrained_sparse,
      opt_ders_unconstrained, opt_costs, n_fixed);

  // Print the resulting coefficients to the console

  Polynomial p0_unC_sparse, p1_unC_sparse, p2_unC_sparse;
  p0_unC_sparse = *polys_unconstrained_sparse[0];
  p1_unC_sparse = *polys_unconstrained_sparse[1];
  p2_unC_sparse = *polys_unconstrained_sparse[2];
  eigen_matlab_dump(p0_unC_sparse);
  eigen_matlab_dump(p1_unC_sparse);
  eigen_matlab_dump(p2_unC_sparse);

  std::cout << p0.eval(0) << std::endl;
  std::cout << p0.eval(0.75) << std::endl;
  std::cout << p2.eval(1) << std::endl;

  std::cout << p0_unC.eval(0) << std::endl;
  std::cout << p0_unC.eval(0.75) << std::endl;
  std::cout << p2_unC.eval(1) << std::endl;

  std::cout << p0_unC_sparse.eval(0) << std::endl;
  std::cout << p0_unC_sparse.eval(0.75) << std::endl;
  std::cout << p2_unC_sparse.eval(1) << std::endl;


  // You should get the following console output:
//  taus=[0.75
//   0.5
//     1];
//
//  p0=[-7.77673e-12
//  -1.05638e-13
//   1.28438e-13
//  -8.06133e-14
//   3.96831e-15
//       44.0379
//      -154.765
//       220.551
//      -149.424
//       40.1398];
//
//  p1=[     0.4
//  0.683407
//  -1.40787
//  -1.30785
//   6.55326
//  -11.4753
//   43.8303
//  -105.226
//   117.954
//  -51.5888];
//
//  p2=[     0.5
//  0.252805
//   1.24804
//    1.2135
//  -5.21775
//   9.50847
//  -25.9649
//   39.5314
//  -26.6531
//   6.58146];

  // Put these values into the following MATLAB script and plot the result:
//  t0_eval = linspace(0,taus(1));
//  x0 = polyval(flipud(p0),t0_eval);
//  t0 = t0_eval;
//
//  t1_eval = linspace(0,taus(2));
//  x1 = polyval(flipud(p1),t1_eval);
//  t1 = t1_eval + taus(1);
//
//  t2_eval = linspace(0,taus(3));
//  x2 = polyval(flipud(p2),t2_eval);
//  t2 = t2_eval + taus(1) + taus(2);
//
//  figure(1),clf,hold on
//  plot(t0,x0)
//  plot(t1,x1)
//  plot(t2,x2)
}

int main(int argc, char ** argv)
{
  srand(time(NULL));

  quad_trajectory_poly_test();
  //  poly_test(argc,argv);
}

