#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <angles/angles.h>
#include <robot_kf/robot_kf.h>

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
    Matrix<double, 3, 2> const B = (Matrix<double, 3, 2>() <<
        0.5 * cos(x_[2]),  0.5 * cos(x_[2]),
        0.5 * sin(x_[2]),  0.5 * sin(x_[2]),
        -1.0 / separation, 1.0 / separation
    ).finished();
    // TODO: Verify that this is correct.
    Matrix3d const cov_process = B * cov_enc * B.transpose();

    predict(enc, cov_process, A_, B);
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
