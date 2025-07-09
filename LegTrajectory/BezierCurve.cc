#include "BezierCurve.hpp"
#include <cmath>
#include <stdexcept>

BezierCurve::BezierCurve()
{

}
BezierCurve::~BezierCurve(){}


VectorXd BezierCurve::getBezierCurve(const MatrixXd& points, 
                              const double& t) {
  int N = points.rows();  // number of Bezier points
  int m = points.cols();  // number of dimensions

  Eigen::VectorXd B(m);
  B.setZero();

  // check the range of t : t in [0, 1]
  if (t < 0 || t > 1) {
    throw std::runtime_error("The parameter t is out of range.");
  }

  for (int j = 0; j < N; ++j) {
    double binomial_coeff = std::tgamma(N) / (std::tgamma(j + 1) * std::tgamma(N - j));
    B += binomial_coeff * std::pow(t, j) * std::pow(1 - t, N - 1 - j) * points.row(j);
  }

  return B;
}
