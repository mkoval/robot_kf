#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <robot_kf/robot_kf.h>

using namespace Eigen;

namespace robot_kf {

KalmanFilter::KalmanFilter(void)
    : x_(Vector3d::Zero())
    , cov_x_(Matrix3d::Zero())
    , A_(Matrix3d::Identity())
    , B_(Matrix3d::Identity())
    , Hgps_((Matrix<double, 2, 3>() << 1, 0, 0, 0, 1, 0).finished())
    , Hcomp_((Matrix<double, 1, 3>() << 0, 0, 1).finished())
    , encoders_prev_(x_)
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

void KalmanFilter::update_encoders(Vector3d encoders_curr, Matrix3d cov_enc)
{
    Vector3d const delta = encoders_curr - encoders_prev_;
    predict(delta, cov_enc, A_, B_);
    encoders_prev_ = encoders_curr;
    normalize_yaw();
}

void KalmanFilter::update_gps(Vector2d gps, Matrix2d cov_gps)
{
    measure(gps, cov_gps, Hgps_);
}

void KalmanFilter::update_compass(double compass, double cov_compass)
{
    Matrix<double, 1, 1> mat_compass, mat_cov_compass;
    mat_compass << compass;
    mat_cov_compass << cov_compass;

    measure(mat_compass, mat_cov_compass, Hcomp_);
    normalize_yaw();
}

template <int m>
void KalmanFilter::predict(Matrix<double, m, 1> u, Matrix<double, m, m> cov_u,
                           Matrix<double, 3, 3> A, Matrix<double, 3, m> B)
{
    x_ = A * x_ + B * u;
    cov_x_ = A * cov_x_ * A.transpose() + cov_u;
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
    x_[2] = fmod(x_[2], 2 * M_PI);
}

};
