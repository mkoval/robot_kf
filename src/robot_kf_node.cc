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

static double const big = 99999.0;

static boost::shared_ptr<tf::TransformListener> sub_tf;
static boost::shared_ptr<tf::TransformBroadcaster> pub_tf;
static ros::Subscriber sub_compass, sub_encoders, sub_gps;
static ros::Publisher pub_fused;

static robot_kf::KalmanFilter kf;
static geometry_msgs::Twist velocity;

static bool watch_compass, watch_encoders, watch_gps;
static std::string global_frame_id, odom_frame_id, base_frame_id;

static void publish(ros::Time stamp)
{
    Eigen::Vector3d const state = kf.getState();

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
        sub_tf->lookupTransform(odom_frame_id, base_frame_id, stamp, t2);
    } catch (tf::TransformException const &e) {
        ROS_WARN("%s", e.what());
        return;
    }

    tf::Transform const t1 = t3 * t2.inverse();
    tf::StampedTransform transform(t1, stamp, global_frame_id, odom_frame_id);
    pub_tf->sendTransform(transform);
    
    // Publish the odometry message.
    nav_msgs::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = global_frame_id;
    msg.child_frame_id = base_frame_id;
    msg.pose.pose =  fused_base.pose;
    // Propagate the covariance forward.
    msg.pose.covariance[0] = -1;
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
    if (watch_compass) publish(msg.header.stamp);
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
    if (watch_encoders) publish(msg.header.stamp);
}

static void updateGps(nav_msgs::Odometry const &msg)
{
    if (msg.header.frame_id != global_frame_id) {
        ROS_ERROR_THROTTLE(10, "GPS message must have frame_id '%s'",
                           global_frame_id.c_str());
        return;
    } else if (msg.child_frame_id != base_frame_id) {
        ROS_ERROR_THROTTLE(10, "GPS message must have child_frame_id '%s'",
                           base_frame_id.c_str());
        return;
    }
    Eigen::Vector2d const z = (Eigen::Vector2d() <<
        msg.pose.pose.position.x, msg.pose.pose.position.y).finished();
    Eigen::Map<Eigen::Matrix<double, 6, 6> const> cov_raw(
        &msg.pose.covariance.front()
    );
    Eigen::Matrix2d const cov = cov_raw.topLeftCorner<2, 2>();

    kf.update_gps(z, cov);

    if (watch_gps) publish(msg.header.stamp);
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
    nh_node.param<bool>("watch_compass",  watch_compass,  true);
    nh_node.param<bool>("watch_encoders", watch_encoders, false);
    nh_node.param<bool>("watch_gps",      watch_gps,      false);
    nh_node.param<std::string>("global_frame_id", global_frame_id, "/map");
    nh_node.param<std::string>("odom_frame_id",   odom_frame_id,   "/odom");
    nh_node.param<std::string>("base_frame_id",   base_frame_id,   "/base_footprint");

    sub_tf = boost::make_shared<tf::TransformListener>();
    pub_tf = boost::make_shared<tf::TransformBroadcaster>();
    sub_compass  = nh.subscribe("compass", 1, &updateCompass);
    sub_encoders = nh.subscribe("wheel_odom", 1, &updateEncoders);
    sub_gps      = nh.subscribe("gps", 1, &updateGps);
    pub_fused    = nh.advertise<nav_msgs::Odometry>("odom_fused", 100);

    ros::spin();
    return 0;
}
