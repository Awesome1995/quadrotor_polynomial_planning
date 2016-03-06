#include "Polynomial.hpp"
#include <stdio.h>

double Polynomial::eval(double t)
{
  double val = 0;
  for (int nn = 0; nn < this->coeffs.rows(); nn++) {
    val += coeffs(nn) * pow(t, nn);
  }
  return val;
}

double Polynomial::HornersEval(double t)
{

  /*
       We want to evaluate the polynomial in x, of coefficients CoefficientsOfPolynomial, using Horner's method.
       The result is stored in dbResult.
   */
  double val = 0.0;
  int i;
  for(i = coeffs.rows() - 1; i >= 0; i--)
  {
    val = val * t + coeffs(i);
  }
  return val;

}

/**
 * evaluates derivative at t
 */
double Polynomial::eval(double t, int derivative) const
{
  double val = 0;
  double prod;
  for (int nn = derivative; nn < this->coeffs.rows(); nn++) {
    prod = 1;
    for (int mm = 0; mm < derivative; mm++) {
      prod *= (nn - mm);
    }
    val += coeffs(nn) * prod * pow(t, nn - derivative);
  }
  return val;
}

Polynomial Polynomial::getDerivative()
{
  Polynomial derivative(this->coeffs.rows() - 2);
  double prod;
  for (int nn = 1; nn < this->coeffs.rows(); nn++) {
    derivative.coeffs(nn - 1) = this->coeffs(nn) * nn;
  }
  return derivative;
}

std::ostream& operator<<(std::ostream& output, const Polynomial & poly)
{
  output << poly.coeffs;
  return output;
}

int sign(int v)
{
  if (v > 0)
    return 1;
  if (v < 0)
    return -1;

  return 0;
}

