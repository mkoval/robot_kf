#ifndef ROBOT_KF_NODE_H_
#define ROBOT_KF_NODE_H_

#include <ros/ros.h>
#include <robot_kf/robot_kf.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <robot_kf/WheelOdometry.h>
#include <Eigen/Geometry>

namespace robot_kf {

class CorrectedKalmanFilter {
public:
    CorrectedKalmanFilter(void);

    void odomCallback(WheelOdometry const &msg);
    void gpsCallback(nav_msgs::Odometry const &gps);
    void compassCallback(sensor_msgs::Imu const &msg);

    Eigen::Vector3d gpsToEigen(nav_msgs::Odometry const &msg);

    KalmanFilter kf_;
    ros::Duration max_latency_;
    std::list<UpdateStep::Ptr> queue_;
};

class UpdateStep {
public:
    typedef boost::shared_ptr<UpdateStep> Ptr;

    UpdateStep(KalmanFilter const &kf, ros::Time stamp);
    ros::Time getStamp(void) const;
    virtual void restore(KalmanFilter &filter) const;
    virtual void update(KalmanFilter &filter) const = 0;

private:
    ros::Time const stamp_;
    Eigen::Vector3d const x_;
    Eigen::Matrix3d const cov_;
};

class GPSUpdateStep : public UpdateStep {
public:
    GPSUpdateStep(nav_msgs::Odometry const &msg);
    virtual void update(KalmanFilter &filter);

private:
    Eigen::Vector3d z_;
    Eigen::Matrix3d cov_;
};

class OdometryUpdateStep : public UpdateStep {
public:
    OdometryUpdateStep(WheelOdometry const &msg);
    virtual void update(KalmanFilter &filter);

private:
    Eigen::Vector2d z_;
    Eigen::Matrix2d cov_;
    double separation_;
};

};

#endif
