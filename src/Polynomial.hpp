#ifndef __poly_hpp__
#define __poly_hpp__

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <iostream>
#include "QuadraticProgramEliminationSolve.h"


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


};


std::ostream& operator<<(std::ostream& output, const Polynomial & poly);

#endif
