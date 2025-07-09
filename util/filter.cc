#include "filter.hpp"

filter::filter(/* args */)
{
}

filter::~filter()
{
}

MatrixXd filter::pinv(const MatrixXd &A)
{
    return A.completeOrthogonalDecomposition().pseudoInverse();
}

double filter::tustin_derivative(double input, double input_old, double output_old, double cutoff_freq)
{
    double time_const = 1 / (2 * M_PI * cutoff_freq);
    double output = 0;

    output = (2 * (input - input_old) - (Ts - 2 * time_const) * output_old) / (Ts + 2 * time_const);

    return output;
}

double filter::lowpassfilter(double input, double input_old, double output_old, double cutoff_freq) 
{
    double time_const = 1 / (2 * M_PI * cutoff_freq);
    double output = 0;

    output = (Ts * (input + input_old) - (Ts - 2 * time_const) * output_old) / (Ts + 2 * time_const);
    return output;
}

Vector3d filter::quat2euler(double qw, double qx, double qy, double qz)
{
    double sinr_cosp = 2 * (qw * qx + qy * qz);
    double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    euler[0] = M_PI / 2 + atan2(sinr_cosp, cosr_cosp);

    double sinp = sqrt(1 + 2 * (qw * qy - qx * qz));
    double cosp = sqrt(1 - 2 * (qw * qy - qx * qz));
    euler[1] = 2 * atan2(sinp, cosp) - M_PI / 2;

    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    euler[2] = atan2(siny_cosp, cosy_cosp);
    
    return euler;

}

Vector3d filter::quat2rpy(Vector4d quat) {
  
  Vector3d rpy;

    rpy(0) = std::atan2(2.0 * (quat[0] * quat[1] + quat[2] * quat[3]),
                        1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    
    rpy(1) = std::asin(2.0 * (quat[0] * quat[2] - quat[3] * quat[1]));

    rpy(2) = std::atan2(2.0 * (quat[0] * quat[3] + quat[1] * quat[2]),
                        1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
    

    // cout << rpy << endl;
    return rpy;
}

Matrix3d filter::xyz_to_zyx_matrix(const Matrix3d& R_xyz, Vector3d& out_zyx_angles) {
    // 1) XYZ 행렬로부터 Rotation 객체(즉, Matrix3d) 그대로 이용
    // 2) ZYX 순서로 오일러 각 추출: eulerAngles(2,1,0) => [z, y, x]
    out_zyx_angles = R_xyz.eulerAngles(2, 1, 0);

    // 3) 추출한 [z, y, x]를 이용해 Z→Y→X 순서로 다시 회전행렬 생성
    AngleAxisd Rz(out_zyx_angles[0], Vector3d::UnitZ());
    AngleAxisd Ry(out_zyx_angles[1], Vector3d::UnitY());
    AngleAxisd Rx(out_zyx_angles[2], Vector3d::UnitX());

    Matrix3d R_zyx;
    R_zyx = Rz * Ry * Rx;
    
    return R_zyx;
}


Matrix3d filter::skew(Vector3d& v)
{
    Matrix3d S;
    S <<  0,    -v(2),  v(1),
          v(2),  0,    -v(0),
         -v(1), v(0),   0;

    return S;
}