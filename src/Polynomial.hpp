#ifndef __poly_hpp__
#define __poly_hpp__

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <iostream>
//#include <eigen_utils/eigen_numerical.hpp>

//#include <lcmtypes/planning_polynomial_t.h>

typedef Eigen::Triplet<double> Trip;

class Polynomial {
public:
  Eigen::VectorXd coeffs;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW


  /**
   * default constructor, order 1 with 0 coefficient
   */
  Polynomial() :
      coeffs(1)
  {
    this->coeffs.setZero();
  }

  /**
   * order N with zero coefficients
   */
  Polynomial(int N) :
      coeffs(N + 1)
  {
    this->coeffs.setZero();
  }

  /**
   * construct with coefficients
   */
  template<typename Derived>
  Polynomial(const Eigen::MatrixBase<Derived> & coeffs) :
      coeffs(coeffs)
  {
  }

//  Polynomial(const planning_polynomial_t * msg) :
//      coeffs(Eigen::Map<const Eigen::VectorXd>(msg->coeffs, msg->N_poly))
//  {
//  }

  /**
   * evaluates the polynomial at t
   */
  double eval(double t);

  /**
   * evaluates the polynomial at t
   */
  double HornersEval(double t);


  /**
   * evaluates derivative at t
   */
  double eval(double t, int derivative);

  /**
   * polulates derivative_values order deterimined by derivative_values.rows().
   * 0 indicates 0th derivative - eval(t) - and so on
   */
//  template<typename Derived>
//  double eval(double t, Eigen::MatrixBase<Derived> & derivative_values)
//  {
//    for (int dd = 0; dd < derivative_values.rows(); dd++) {
//      derivative_values(dd) = this->eval(t, dd);
//    }
//  }

  /**
   * returns the derivative polynomial
   */
  Polynomial getDerivative();

  /**
   * scale the independent axis such that poly.eval(t)=poly_scaled(scale*t)
   */
  void scaleIndep(double scale)
  {
    double prod = 1;
    for (int ii = 1; ii < this->coeffs.rows(); ii++) {
      prod *= scale;
      this->coeffs(ii) /= prod;
    }
  }

  /**
   * scale the dependent axis such that scaling * poly.eval(t) = poly_scaled.eval(t)
   */
  void scaleDep(double scale)
  {
    this->coeffs *= scale;
  }

  void setCoeffs(Eigen::VectorXd const& coeffs) {
      this->coeffs = coeffs;
  }

//  void to_planning_polynomial_t(planning_polynomial_t * msg)
//  {
//    msg->N_poly = this->coeffs.rows();
//    msg->coeffs = (double *) calloc(msg->N_poly, sizeof(double));
//    memcpy(msg->coeffs, this->coeffs.data(), this->coeffs.rows() * sizeof(double));
//  }

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


template<typename DerivedQ, typename DerivedA, typename Derivedb, typename Derivedx>
void quadProgEliminationSolve(const Eigen::MatrixBase<DerivedQ> & Q, const Eigen::MatrixBase<DerivedA> & A,
    const Eigen::MatrixBase<Derivedb> & b, Eigen::MatrixBase<Derivedx> & x_star)
{
  int m = A.rows();
  int n = A.cols();
  assert(n>=m);

  assert(Q.cols()==Q.rows());
  assert(Q.cols()==n);
  assert(b.rows()==m);
  assert(b.cols()==1);
  assert(x_star.cols()==1);
  assert(x_star.rows()==n);

  if (m == n) {

//    Eigen::ColPivHouseholderQR<Eigen::MatrixBase<DerivedA> > A_decomp;
//    A_decomp.compute(A);
//    if (!A_decomp.isInvertible()) {
//      fprintf(stderr,
//          "Warning: A matrix for fully constrained quadProgEliminationSolve in %s is not invertible, line %d\n",
//          __FILE__, __LINE__);
//    }
    //    x_star = A_decomp.solve(b);

    if (!A.colPivHouseholderQr().isInvertible()) {
      fprintf(stderr,
          "Warning: A matrix for fully constrained quadProgEliminationSolve in %s is not invertible, line %d\n",
          __FILE__, __LINE__);
    }
    x_star = A.colPivHouseholderQr().solve(b);
  }
  else {
    Eigen::MatrixXd B = A.leftCols(m);
    Eigen::MatrixXd R = A.rightCols(n - m);

    Eigen::MatrixXd Q_BB = Q.topLeftCorner(m, m);
    Eigen::MatrixXd Q_BR = Q.topRightCorner(m, n - m);
    Eigen::MatrixXd Q_RB = Q.bottomLeftCorner(n - m, m);
    Eigen::MatrixXd Q_RR = Q.bottomRightCorner(n - m, n - m);

    /*
     *  ColPivHouseholderQR (faster, less accurate)
     *  FullPivHouseholderQR (slower, more accurate)
     *  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> B_decomp;
     *  Eigen::FullPivHouseholderQR<Eigen::MatrixXd> B_decomp;
     */

    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> B_decomp;
    B_decomp.compute(B);

    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> B_transpose_decomp;
    B_transpose_decomp.compute(B.transpose());
    if (!B_decomp.isInvertible()) {
      fprintf(stderr, "Warning: B matrix in A=[B R] for quadProgEliminationSolve in %s is not invertible, line %d\n",
          __FILE__, __LINE__);
    }
    Eigen::MatrixXd B_inv_R = B_decomp.solve(R);

    //f_prime_RR=2*b'*(B'\(Q_BR-(Q_BB/B)*R));
    Eigen::VectorXd f_prime_RR;
//    eigen_dump(b);
//    eigen_dump(Q_BR);
//    eigen_dump(R);
//    eigen_dump(Q_BB);
//    eigen_dump(B_inv_R);
    f_prime_RR = (2 * b.transpose() * (B_transpose_decomp.solve(Q_BR - Q_BB * B_inv_R))).transpose();

    //    Q_prime_RR=...
    //          Q_RR+R'*(B'\Q_BB/B)*R...
    //          -(R'/B')*Q_BR...
    //          -(Q_RB/B)*R;
    Eigen::MatrixXd Q_prime_RR = Q_RR + B_inv_R.transpose() * Q_BB * B_inv_R - B_inv_R.transpose() * Q_BR
        - Q_RB * B_inv_R;

    /*
     *  ColPivHouseholderQR (faster, less accurate)
     *  FullPivHouseholderQR (slower, more accurate)
     *  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> Q_prime_RR_decomp;
     *  Eigen::FullPivHouseholderQR<Eigen::MatrixXd> Q_prime_RR_decomp;
     */
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> Q_prime_RR_decomp;
    Q_prime_RR_decomp.compute(Q_prime_RR);
    if (!Q_prime_RR_decomp.isInvertible()) {
      fprintf(stderr, "Warning: Q_primer_RR matrix in quadProgEliminationSolve in %s is not invertible, line %d\n",
          __FILE__, __LINE__);
    }

    //    x_R_star = -(2*Q_prime_RR)\f_prime_RR';
    //    x_B_star = B\(b-R*x_R_star);
    //    x_star = [x_B_star;x_R_star];
    x_star.bottomRows(n - m) = -0.5 * Q_prime_RR_decomp.solve(f_prime_RR);
    x_star.topRows(m) = B_decomp.solve(b - R * x_star.bottomRows(n - m));
  }
}




std::ostream& operator<<(std::ostream& output, const Polynomial & poly);

#endif
