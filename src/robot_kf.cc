#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>

using namespace Eigen;

namespace robot_kf {

class KalmanFilter {
public:
    KalmanFilter(void)
        : x_(Vector3d::Zero())
        , cov_x_(Matrix3d::Zero())
        , A_(Matrix3d::Identity())
        , B_(Matrix3d::Identity())
        , Hgps_((Matrix<double, 2, 3>() << 1, 0, 0, 0, 1, 0).finished())
        , Hcomp_((Matrix<double, 1, 3>() << 0, 0, 1).finished())
        , encoders_prev_(x_)
    {}

    virtual ~KalmanFilter(void)
    {}

    Vector3d getState(void) const
    {
        return x_;
    }

    Matrix3d getCovariance(void) const
    {
        return cov_x_;
    }

    template <int m>
    void predict(Matrix<double, m, 1> u, Matrix<double, m, m> cov_u,
                 Matrix3d A, Matrix<double, 3, m> B)
    {
        x_ = A * x_ + B * u;
        cov_x_ = A * cov_x_ * A.transpose() + cov_u;
    }

    template <int m>
    void measure(Matrix<double, m, 1> z, Matrix<double, m, m> cov_z,
                 Matrix<double, m, 3> H)
    {
        Matrix<double, 3, m> const K = cov_x_ * H.transpose() * (H * cov_x_
                                      * H.transpose() + cov_z).inverse();
        x_ = x_ + K * (z - H * x_);
        cov_x_ = (Matrix3d::Identity() - K * H) * cov_x_;
    }

    void update_encoders(Vector3d encoders_curr, Matrix3d cov_enc)
    {
        Vector3d const delta = encoders_curr - encoders_prev_;
        predict(delta, cov_enc, A_, B_);
        encoders_prev_ = encoders_curr;
        normalize_yaw();
    }

    void update_gps(Vector2d gps, Matrix2d cov_gps)
    {
        measure(gps, cov_gps, Hgps_);
    }

    void update_compass(double compass, double cov_compass)
    {
        Matrix<double, 1, 1> mat_compass, mat_cov_compass;
        mat_compass << compass;
        mat_cov_compass << cov_compass;

        measure(mat_compass, mat_cov_compass, Hcomp_);
        normalize_yaw();
    }

private:
    Vector3d x_;
    Matrix3d cov_x_;
    Matrix3d const A_, B_;
    Matrix<double, 2, 3> const Hgps_;
    Matrix<double, 1, 3> const Hcomp_;
    Vector3d encoders_prev_;

    void normalize_yaw(void) {
        x_[2] = fmod(x_[2], 2 * M_PI);
    }
};

};

int main(void)
{
    return 0;
}
