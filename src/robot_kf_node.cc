#include <Eigen/Geometry>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <robot_kf/WheelOdometry.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <robot_kf/robot_kf.h>

using Eigen::Vector3d;
using Eigen::Matrix3d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

static double const big = 99999.0;

static boost::shared_ptr<tf::TransformListener> sub_tf;
static boost::shared_ptr<tf::TransformBroadcaster> pub_tf;
static ros::Subscriber sub_compass, sub_encoders, sub_gps;
static ros::Publisher pub_fused;

static robot_kf::KalmanFilter kf;
static geometry_msgs::Twist velocity;

static std::string global_frame_id, odom_frame_id, base_frame_id, offset_frame_id;
static boost::shared_ptr<tf::Transform> offset_tf;

static void publish(ros::Time stamp)
{
    Vector3d const state = kf.getState();
    Matrix3d const cov = kf.getCovariance();

    // Wrap the fused state estimate in a ROS message.
    geometry_msgs::PoseStamped fused_base;
    fused_base.header.stamp    = stamp;
    fused_base.header.frame_id = odom_frame_id;
    fused_base.pose.position.x = state[0];
    fused_base.pose.position.y = state[1];
    fused_base.pose.position.z = 0.0;
    fused_base.pose.orientation = tf::createQuaternionMsgFromYaw(state[2]);

    /*
     * We actually want to publish the /map to /base_footprint transform, but
     * this would cause /base_footprint to have two parents. Instead, we need
     * to publish the /map to /odom transform. See this chart:
     *
     * /map --[T1]--> /odom --[T2]--> /base_footprint
     * /map ----------[T3]----------> /base_footprint
     *
     * The output of the Kalman filter is T3, but we want T1. We find T1 by
     * computing T1 = T3 * inv(T2), where T2 is provided by the odometry
     * source.
     */
    tf::Vector3 const pos(state[0], state[1], 0);
    tf::Quaternion const ori = tf::createQuaternionFromYaw(state[2]);
    tf::Transform t3(ori, pos);

    tf::StampedTransform t2;
    try {
        sub_tf->waitForTransform(odom_frame_id, base_frame_id, stamp, ros::Duration(1.0));
        sub_tf->lookupTransform(odom_frame_id, base_frame_id, stamp, t2);
    } catch (tf::TransformException const &e) {
        ROS_WARN("%s", e.what());
        return;
    }

    tf::Transform const t1 = t3 * t2.inverse();
    tf::StampedTransform transform(t1, stamp, global_frame_id, odom_frame_id);
    pub_tf->sendTransform(transform);

    // Relative frame to fix RViz numerical stability issues.
    if (offset_tf) {
        tf::StampedTransform offset_stamped(*offset_tf, stamp, global_frame_id, offset_frame_id);
        pub_tf->sendTransform(offset_stamped);
    }
    
    // Publish the odometry message.
    nav_msgs::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = global_frame_id;
    msg.child_frame_id = base_frame_id;
    msg.pose.pose =  fused_base.pose;

    // Use the covariance estimated by the filter.
    Eigen::Map<Matrix6d> cov_out(&msg.pose.covariance.front());
    cov_out.topLeftCorner<2, 2>() = cov.topLeftCorner<2, 2>();
    cov_out(5, 5) = cov(2, 2);

    // TODO: Estimate the covariance of the encoders.
    msg.twist.twist = velocity;
    msg.twist.covariance[0] = -1;
    pub_fused.publish(msg);
}

static void updateCompass(sensor_msgs::Imu const &msg)
{
    if (msg.header.frame_id != base_frame_id) {
        ROS_ERROR_THROTTLE(10, "Imu message must have frame_id '%s'",
                           base_frame_id.c_str());
        return;
    }

    double const yaw = tf::getYaw(msg.orientation);
    Eigen::Map<Eigen::Matrix3d const> cov_raw(&msg.orientation_covariance.front());

    kf.update_compass(yaw, cov_raw(2, 2));
}

static void updateEncoders(robot_kf::WheelOdometry const &msg)
{
    if (msg.header.frame_id != base_frame_id) {
        ROS_ERROR_THROTTLE(10, "WheelOdometry message must have frame_id '%s'", base_frame_id.c_str());
        return;
    } else if (msg.separation <= 0) {
        ROS_ERROR_THROTTLE(10, "Wheel separation in WheelOdometry message must be positive.");
        return;
    }

    Eigen::Vector2d const z = (Eigen::Vector2d() <<
        msg.left.movement, msg.right.movement).finished();
    Eigen::Matrix2d const cov_z = (Eigen::Matrix2d() <<
        msg.left.variance, 0.0, 0.0, msg.right.variance).finished();

    // Compute velocity using the wheel encoders. Theoretically the GPS and
    // compass could also be used to estimate these velocities, but those
    // estimates are too noisy to justify the complexity of adding two more
    // state variables.
    double const movement_linear = (z[1] + z[0]) / 2;
    double const movement_angular = (z[1] - z[0]) / msg.separation;
    double const delta_time = msg.timestep.toSec();
    velocity.linear.x = movement_linear / delta_time;
    velocity.angular.z = movement_angular / delta_time;

    kf.update_encoders(z, cov_z, msg.separation);
    publish(msg.header.stamp);
}

static void updateGps(nav_msgs::Odometry const &msg)
{
    if (msg.header.frame_id != global_frame_id) {
        ROS_ERROR_THROTTLE(10, "GPS message must have frame_id '%s'",
                           global_frame_id.c_str());
        return;
    }

    Eigen::Vector2d const z_raw = (Eigen::Vector2d() <<
        msg.pose.pose.position.x, msg.pose.pose.position.y).finished();
    Eigen::Map<Eigen::Matrix<double, 6, 6> const> cov6_raw(
        &msg.pose.covariance.front()
    );
    Eigen::Matrix2d const cov_raw = cov6_raw.topLeftCorner<2, 2>();

    /* The GPS gives a position relative to child_frame_id, which is located at
     * the center of the GPS antennna. However, the Kalman filter only manipulates
     * quantities in base_frame_id. So the situation is:
     *
     *     /map --[ T1 ]--> /gps <--[ T2 ]-- /base_footprint
     *     /map -----------[ T3 ]----------> /base_footprint
     *
     * Transformation T1 is provided by the GPS, T2 is a static transformation
     * provided by the URDF, and T3 is the desired translation. Therefore, we
     * can find T3 by: T3 = T1 * T2^{-1}.
     */
    tf::StampedTransform t2_inv;
    try {
        sub_tf->lookupTransform(msg.child_frame_id, base_frame_id, ros::Time(0), t2_inv);
    } catch (tf::TransformException const &e) {
        ROS_WARN("%s", e.what());
        return;
    }

    tf::Vector3 const z_bt_raw(z_raw[0], z_raw[1], 0);
    tf::Vector3 const z_bt = t2_inv * z_bt_raw;
    Eigen::Vector2d const z = (Eigen::Vector2d() << z_bt[0], z_bt[1]).finished();
    // TODO: Transform the covariance matrix using the rotation matrix.
    Eigen::Matrix2d const cov = cov_raw;

    if (!offset_tf) {
        tf::Vector3 const pos_offset(z[0], z[1], 0.0);
        tf::Quaternion const ori_offset(0.0, 0.0, 0.0, 1.0);
        offset_tf = boost::make_shared<tf::Transform>(ori_offset, pos_offset);
        ROS_INFO("Initialized %s to (%f, %f)", offset_frame_id.c_str(), z[0], z[1]);
    }

    kf.update_gps(z, cov);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_kf_node");

    // Initialize the skeleton twist message.
    velocity.linear.y = 0;
    velocity.linear.z = 0;
    velocity.angular.x = 0;
    velocity.angular.y = 0;

    ros::NodeHandle nh, nh_node("~");
    nh_node.param<std::string>("global_frame_id", global_frame_id, "/map");
    nh_node.param<std::string>("odom_frame_id", odom_frame_id, "/odom");
    nh_node.param<std::string>("base_frame_id", base_frame_id, "/base_footprint");
    nh_node.param<std::string>("offset_frame_id", offset_frame_id, "/map_offset");

    sub_tf = boost::make_shared<tf::TransformListener>();
    pub_tf = boost::make_shared<tf::TransformBroadcaster>();
    sub_compass  = nh.subscribe("compass", 1, &updateCompass);
    sub_encoders = nh.subscribe("wheel_odom", 1, &updateEncoders);
    sub_gps      = nh.subscribe("gps", 1, &updateGps);
    pub_fused    = nh.advertise<nav_msgs::Odometry>("odom_fused", 100);

    ros::spin();
    return 0;
}
