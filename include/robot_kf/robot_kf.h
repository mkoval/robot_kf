#ifndef ROBOT_KF_H_
#define ROBOT_KF_H_

#include <Eigen/Core>

namespace robot_kf {

class KalmanFilter {
public:
    KalmanFilter(void);
    virtual ~KalmanFilter(void);

    Eigen::Vector3d getState(void) const;
    Eigen::Matrix3d getCovariance(void) const;

    void update_encoders(Eigen::Vector3d encoders_curr, Eigen::Matrix3d cov_enc);
    void update_gps(Eigen::Vector2d gps, Eigen::Matrix2d cov_gps);
    void update_compass(double compass, double cov_compass);

private:
    Eigen::Vector3d x_;
    Eigen::Matrix3d cov_x_;
    Eigen::Matrix3d const A_, B_;
    Eigen::Matrix<double, 2, 3> const Hgps_;
    Eigen::Matrix<double, 1, 3> const Hcomp_;
    Eigen::Vector3d encoders_prev_;

    template <int m>
    void predict(Eigen::Matrix<double, m, 1> u,
                 Eigen::Matrix<double, m, m> cov_u,
                 Eigen::Matrix<double, 3, 3> A,
                 Eigen::Matrix<double, 3, m> B);

    template <int m>
    void measure(Eigen::Matrix<double, m, 1> z,
                 Eigen::Matrix<double, m, m> cov_z,
                 Eigen::Matrix<double, m, 3> H);

    void normalize_yaw(void);
};

};

#endif
