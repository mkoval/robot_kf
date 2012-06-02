#ifndef ROBOT_KF_NODE_H_
#define ROBOT_KF_NODE_H_

#include <ros/ros.h>
#include <robot_kf/robot_kf.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <robot_kf/WheelOdometry.h>
#include <Eigen/Geometry>

namespace robot_kf {

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
    GPSUpdateStep(KalmanFilter const &, ros::Time stamp,
                  Eigen::Vector3d z, Eigen::Matrix3d cov);
    virtual void update(KalmanFilter &filter) const;

private:
    Eigen::Vector3d z_;
    Eigen::Matrix3d cov_;
};

class OdometryUpdateStep : public UpdateStep {
public:
    OdometryUpdateStep(KalmanFilter const &, WheelOdometry const &msg);
    virtual void update(KalmanFilter &filter) const;

private:
    Eigen::Vector2d z_;
    Eigen::Matrix2d cov_;
    double separation_;
};

class CorrectedKalmanFilter {
public:
    CorrectedKalmanFilter(double seconds,
        std::string local_frame_id,
        std::string odom_frame_id,
        std::string global_frame_id);
    void init(std::string topic_odom, std::string topic_gps,
              std::string topic_compass, std::string topic_fused);

    void odomCallback(WheelOdometry const &msg);
    void gpsCallback(nav_msgs::Odometry const &gps);
    void compassCallback(sensor_msgs::Imu const &msg);

    void publish(ros::Time stamp, Eigen::Vector3d state, Eigen::Matrix3d cov);

private:
    void pruneUpdates(ros::Time stamp);

    KalmanFilter kf_;
    ros::Duration max_latency_;
    std::list<UpdateStep::Ptr> queue_;

    std::string base_frame_id_;
    std::string odom_frame_id_;
    std::string global_frame_id_;

    ros::NodeHandle nh_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_gps_;
    ros::Subscriber sub_compass_;
    ros::Publisher pub_fused_;
    tf::TransformListener sub_tf;
    tf::TransformBroadcaster pub_tf;
};

};

#endif
