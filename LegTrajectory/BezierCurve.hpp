#pragma once

#include "globals.hpp"

using namespace std;

class BezierCurve
{
private:
    MatrixXd Derivative_points;
    double Bezier_Order = 6;
public:
    BezierCurve();
    ~BezierCurve();
    VectorXd getBezierCurve(const MatrixXd& points, const double& t);
     
};
