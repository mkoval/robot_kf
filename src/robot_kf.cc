#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <angles/angles.h>
#include <robot_kf/robot_kf.h>

#include <iostream>

using namespace Eigen;

namespace robot_kf {

KalmanFilter::KalmanFilter(void)
    : x_(Vector3d::Zero())
    , cov_x_(9999 * Matrix3d::Identity())
    , A_(Matrix3d::Identity())
    , Hgps_((Matrix<double, 2, 3>() << 1, 0, 0, 0, 1, 0).finished())
    , Hcomp_((Matrix<double, 1, 3>() << 0, 0, 1).finished())
{}

KalmanFilter::~KalmanFilter(void)
{}

Vector3d KalmanFilter::getState(void) const
{
    return x_;
}

Matrix3d KalmanFilter::getCovariance(void) const
{
    return cov_x_;
}

void KalmanFilter::update_encoders(Vector2d enc, Matrix2d cov_enc, double separation)
{
    /* This is non-linear in theta because of the x and y elements:
     *      x = (r + l)/2 * cos(theta + (r - l) / (2s))
     *      y = (r + l)/2 * sin(theta + (r - l) / (2s))
     *  theta = (r - l)/(2s)
     * We can linearize this around the current state estimate using the
     * Jacobians A = Jac(f, x) and W = Jac(f, u).
     */
    double const dlinear = (enc[1] + enc[0]) / 2;
    double const dtheta  = (enc[1] - enc[0]) / separation;
    double const theta_halfway = x_[2] + dtheta / 2;

    Matrix3d const A = (Matrix3d() <<
        1, 0, -dlinear * sin(theta_halfway),
        0, 1, +dlinear * cos(theta_halfway),
        0, 0, 1).finished();
    Matrix<double, 3, 2> const W = (Matrix<double, 3, 2>() <<
        +dlinear * sin(theta_halfway) + 0.5 * cos(theta_halfway),
        -dlinear * sin(theta_halfway) + 0.5 * cos(theta_halfway),
        -dlinear * cos(theta_halfway) + 0.5 * sin(theta_halfway),
        +dlinear * cos(theta_halfway) + 0.5 * sin(theta_halfway),
        -1.0 / separation,
        +1.0 / separation).finished();

    x_ += (Vector3d() <<
            dlinear * cos(theta_halfway),
            dlinear * sin(theta_halfway),
            dtheta).finished();
    cov_x_ = A * cov_x_ * A.transpose() + W * cov_enc * W.transpose();
    normalize_yaw();
}

void KalmanFilter::update_gps(Vector2d gps, Matrix2d cov_gps)
{
    measure(gps, cov_gps, Hgps_);
}

void KalmanFilter::update_compass(double compass, double cov_compass)
{
    // Renormalize the compass heading so the filter behaves correctly when
    // crossing +/-pi. This prevents the heading from slowly drifting the "long
    // way" around the circle (i.e. through 0 instead of +/-pi).
    if (x_[2] - compass > M_PI) {
        compass = compass + 2 * M_PI;
    } else if (x_[2] - compass < -M_PI) {
        compass = compass - 2 * M_PI;
    }

    Matrix<double, 1, 1> mat_compass, mat_cov_compass;
    mat_compass << compass;
    mat_cov_compass << cov_compass;

    measure(mat_compass, mat_cov_compass, Hcomp_);
    normalize_yaw();
} 

template <int m>
void KalmanFilter::predict(Matrix<double, m, 1> u, Matrix<double, 3, 3> cov_process,
                           Matrix<double, 3, 3> A, Matrix<double, 3, m> B)
{
    x_ = A * x_ + B * u;
    cov_x_ = A * cov_x_ * A.transpose() + cov_process;
}

template <int m>
void KalmanFilter::measure(Matrix<double, m, 1> z, Matrix<double, m, m> cov_z,
                           Matrix<double, m, 3> H)
{
    Matrix<double, 3, m> const K = cov_x_ * H.transpose() * (H * cov_x_
                                  * H.transpose() + cov_z).inverse();
    x_ = x_ + K * (z - H * x_);
    cov_x_ = (Matrix3d::Identity() - K * H) * cov_x_;
}

void KalmanFilter::normalize_yaw(void) {
    x_[2] = angles::normalize_angle(x_[2]);
}

};
