#include <Eigen/Geometry>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <robot_kf/robot_kf.h>
#include <robot_kf/WheelOdometry.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "robot_kf_node.h"

using Eigen::Affine3d;
using Eigen::Map;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

namespace robot_kf {

/*
 * CorrectedKalmanFilter - corrects for compass latency.
 */
CorrectedKalmanFilter::CorrectedKalmanFilter(
        double seconds,
        std::string base_frame_id,
        std::string odom_frame_id,
        std::string global_frame_id)
    : max_latency_(seconds)
    , base_frame_id_(base_frame_id)
    , odom_frame_id_(odom_frame_id)
    , global_frame_id_(global_frame_id)
{
}

void CorrectedKalmanFilter::init(std::string topic_odom, std::string topic_gps,
                                 std::string topic_compass, std::string topic_fused)
{
    sub_odom_ = nh_.subscribe(topic_odom, 10, &CorrectedKalmanFilter::odomCallback, this);
    sub_gps_ = nh_.subscribe(topic_gps, 1, &CorrectedKalmanFilter::gpsCallback, this);
    sub_compass_ = nh_.subscribe(topic_compass, 1, &CorrectedKalmanFilter::compassCallback, this);
    pub_fused_ = nh_.advertise<nav_msgs::Odometry>(topic_fused, 10);
}

void CorrectedKalmanFilter::odomCallback(WheelOdometry const &msg)
{
    if (msg.header.frame_id != base_frame_id_) {
        ROS_ERROR_THROTTLE(10, "Expected odometry to have frame %s, but actually has %s",
            base_frame_id_.c_str(), msg.header.frame_id.c_str());
        return;
    }

    UpdateStep::Ptr action = boost::make_shared<OdometryUpdateStep>(kf_, msg);
    queue_.push_back(action);
    action->update(kf_);
    pruneUpdates(msg.header.stamp);
}

void CorrectedKalmanFilter::gpsCallback(nav_msgs::Odometry const &msg)
{
    if (msg.header.frame_id != global_frame_id_) {
        ROS_ERROR_THROTTLE(10, "Expected GPS to have frame %s, but actually has %s.",
            global_frame_id_.c_str(), msg.header.frame_id.c_str());
        return;
    }

    geometry_msgs::Point const &pos = msg.pose.pose.position;
    Vector3d const z_offset = (Vector3d() << pos.x, pos.y, pos.z).finished();
    Map<Matrix6d const> const cov6_offset_raw(&msg.pose.covariance.front());
    Matrix3d const cov_offset = cov6_offset_raw.topLeftCorner<3, 3>();

    // Transform from the GPS frame to the base frame.
    tf::StampedTransform t2_inv;
    try {
        sub_tf.lookupTransform(msg.child_frame_id, base_frame_id_,
                                msg.header.stamp, t2_inv);
    } catch (tf::TransformException const &e) {
        ROS_WARN_THROTTLE(10, "%s", e.what());
        return;
    }

    Affine3d transform;
    tf::TransformTFToEigen(t2_inv, transform);
    Vector3d z = transform * z_offset;
    Matrix3d cov = transform.linear() * cov_offset * transform.linear().transpose();

    ros::Time stamp = msg.header.stamp;
    UpdateStep::Ptr action = boost::make_shared<GPSUpdateStep>(kf_, stamp, z, cov);
    queue_.push_back(action);
    action->update(kf_);
    pruneUpdates(stamp);
}

void CorrectedKalmanFilter::compassCallback(sensor_msgs::Imu const &msg)
{
    if (msg.header.frame_id != global_frame_id_) {
        ROS_ERROR_THROTTLE(10, "Expected compass to have frame %s, but actually has %s.",
            global_frame_id_.c_str(), msg.header.frame_id.c_str());
        return;
    }

    double const z = tf::getYaw(msg.orientation);
    double const var = msg.orientation_covariance[8];

    // Find the first update step that occured after this measurement.
    std::list<UpdateStep::Ptr>::iterator it = queue_.end();
    while (it != queue_.begin() && (*it)->getStamp() > msg.header.stamp) {
        --it;
    }

    // There aren't any updates to unwind, so we can proceed as normal.
    if (it == queue_.end()) {
        kf_.update_compass(z, var);
    }
    // We've hit the end of the queue and still haven't found the first
    // update to unwind. This means the queue wasn't long enough.
    else if (it == queue_.begin()) {
        ROS_WARN_THROTTLE(10, "Skipping compass update that exceeds maximum latency");
        return;
    }
    // Otherwise, we can unwind the updates, apply the compass update, then
    // replay the unwound updates.
    else {
        ++it;
        (*it)->restore(kf_);
        kf_.update_compass(z, var);

        for (++it; it != queue_.end(); ++it) {
            (*it)->update(kf_);
        }
    }
}

void CorrectedKalmanFilter::pruneUpdates(ros::Time stamp)
{
    std::list<UpdateStep::Ptr>::iterator it = queue_.begin();
    while (it != queue_.end() && stamp - (*it)->getStamp() > max_latency_) {
        it = queue_.erase(it);
    }
}

/*
 * UpdateStep - abstract base class
 */
UpdateStep::UpdateStep(KalmanFilter const &kf, ros::Time stamp)
    : stamp_(stamp)
    , x_(kf.getState())
    , cov_(kf.getCovariance())
{}

ros::Time UpdateStep::getStamp(void) const
{
    return stamp_;
}

void UpdateStep::restore(KalmanFilter &kf) const
{
    kf.setState(x_, cov_);
}

/*
 * GPSUpdateStep - concrete implementation of UpdateStep
 */
GPSUpdateStep::GPSUpdateStep(KalmanFilter const &kf, ros::Time stamp,
                             Eigen::Vector3d z, Eigen::Matrix3d cov)
    : UpdateStep(kf, stamp)
    , z_(z)
    , cov_(cov)
{
}

void GPSUpdateStep::update(KalmanFilter &kf) const
{
    kf.update_gps(z_.head<2>(), cov_.topLeftCorner<2, 2>());
}

/*
 * OdometryUpdateStep - concrete implementation of UpdateStep
 */
OdometryUpdateStep::OdometryUpdateStep(KalmanFilter const &kf, WheelOdometry const &msg)
    : UpdateStep(kf, msg.header.stamp)
    , separation_(msg.separation)
{
    z_ << msg.left.movement, msg.right.movement;
    cov_ << msg.left.variance, 0.0, 0.0, msg.right.variance;
}

void OdometryUpdateStep::update(KalmanFilter &kf) const
{
    kf.update_encoders(z_, cov_, separation_);
}

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_kf_node");

    ros::NodeHandle nh_node("~");
    std::string base_frame_id, odom_frame_id, global_frame_id, offset_frame_id;
    double max_latency;
    nh_node.param<std::string>("base_frame_id", base_frame_id, "/base_footprint");
    nh_node.param<std::string>("odom_frame_id", odom_frame_id, "/odom");
    nh_node.param<std::string>("global_frame_id", global_frame_id, "/map");
    nh_node.param<std::string>("offset_frame_id", offset_frame_id, "/map_offset");
    nh_node.param<double>("max_latency", max_latency, 1.0);

    robot_kf::CorrectedKalmanFilter kf(max_latency, base_frame_id, odom_frame_id, global_frame_id);
    kf.init("wheel_odom", "gps", "compass", "odom_fused");

#if 0
    // Initialize the skeleton twist message.
    velocity.linear.y = 0;
    velocity.linear.z = 0;
    velocity.angular.x = 0;
    velocity.angular.y = 0;


    sub_tf = boost::make_shared<tf::TransformListener>();
    pub_tf = boost::make_shared<tf::TransformBroadcaster>();
    sub_compass  = nh.subscribe("compass", 1, &updateCompass);
    sub_encoders = nh.subscribe("wheel_odom", 1, &updateEncoders);
    sub_gps      = nh.subscribe("gps", 1, &updateGps);
    pub_fused    = nh.advertise<nav_msgs::Odometry>("odom_fused", 100);
#endif

    ros::spin();
    return 0;
}
