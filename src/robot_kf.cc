#include <cmath>
#include <Eigen/Dense>
#include <Eigen/LU>

using namespace Eigen;

namespace robot_kf {

template <typename T, int n>
class KalmanFilter {
public:
    KalmanFilter(void)
        : x_ = Vector3d::zeros()
        , sigma_ = Matrix3d::zeros()
    {}

    virtual ~KalmanFilter(void)
    {}

    template <int un>
    virtual void predict(Vector<T, un> u, Matrix<T, un, un> cov_u,
                         Matrix<T, n, n> A, Matrix<T, n, m> B)
    {
        x_ = A * x_ + B * u;
        cov_ = A * cov_x_ * A.transpose() + cov_u;
    }

    template <int zn>
    virtual void measure(Vector<T, zn> z, Matrix<T, zn, zn> cov_z,
                         Matrix<T, zn, n> H)
    {
        Matrix<double, n, zn> const K = cov_x_ * H.transpose() * (H * cov_x_
                                      * H.transpose() + cov_z).inverse();
        x_ = x_ + K * (z - H * x_);
        cov_x_ = (Matrix3d::identity() - K * H) * cov_x_;
    }

    virtual Vector<T, n> getState(void) const
    {
        return x_;
    }

    virtual Matrix<T, n, n> getCovariance(void) const
    {
        return cov_x_;
    }

private:
    Vector<T, n> x_;
    Matrix<T, n, n> cov_x_;
};

template <typename T>
class PositionKalmanFilter : public KalmanFilter<T, 3>
{
public:
    PositionKalmanFilter(void)
        : KalmanFilter()
        , A_ = Matrix<T, 3, 3>::identity()
        , B_ = (Matrix<T, 3, 6>() << -Matrix<T, 3, 3>::identity(),
                                      Matrix<T, 3, 3>::identity())
        , Hgps_ = (Matrix<T, 3, 2>() << 1, 0, 0, 0, 1, 0)
        , Hcomp_ = (Matrix<T, 3, 1>() << 0, 0, 1)
        , encoders_prev_ = x_
    {}

    virtual ~PositionKalmanFilter(void)
    {}

    template <int un>
    virtual void predict(Vector<T, un> u, Matrix<T, un, un> cov_u,
                         Matrix<T, 3, 3> A, Matrix<T, 3, un> B)
    {
        KalmanFilter::predict<un>(u, cov_u, A, B);
        renormalize();
    }

    template <int zn>
    virtual void measure(Vector<T, zn> z, Matrix<T, zn, zn> cov_z,
                         Matrix<T, zn, 3> H)
    {
        KalmanFilter::measure<zn>(z, cov_z, H);
        renormalize();
    }

    virtual void update_encoders(Vector<T, 3> encoders_curr, Matrix<T, 3, 3> cov_enc)
    {
        Vector<T, 6> const u = (Vector<T, 6> << encoders_prev, encoders_curr);
        predict(u, cov_enc, A_, B_);
        encoders_prev_ = encoders_curr;
    }

    virtual void update_gps(Vector<T, 2> gps, Matrix<T, 2, 2> cov_gps)
    {
        measure(gps, cov_gps, Hgps_);
    }

    virtual void update_compass(double compass, cov_compass)
    {
        measure(compass, cov_compass, Hcomp_);
    }

private:
    Matrix<double, 3, 3> const A_;
    Matrix<double, 3, 6> const B_;
    Matrix<double, 2, 3> const Hgps_;
    Matrix<double, 1, 3> const Hcomp_;
    Matrix3f encoders_prev_;

    void renormalize(void) {
        x_[2] = fmod(x_[2], 2 * M_PI);
    }
};

int main(void)
{
}

};
