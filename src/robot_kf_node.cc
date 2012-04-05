#include <Eigen/Geometry>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <robot_kf/robot_kf.h>


static ros::Subscriber sub_compass;
static ros::Subscriber sub_encoders;
static ros::Subscriber sub_gps;
static ros::Publisher  pub_fused;
static robot_kf::KalmanFilter kf;

static bool watch_compass;
static bool watch_encoders;
static bool watch_gps;

static void publish(void)
{
}

static void updateCompass(sensor_msgs::Imu const &msg)
{
    // TODO: TF frame conversion.

    double const yaw = tf::getYaw(msg.orientation);
    double const cov = msg.orientation_covariance[8];

    kf.update_compass(yaw, cov);
    if (watch_compass) publish();
}

static void updateEncoders(nav_msgs::Odometry const &msg)
{
    // TODO: TF frame conversion;

    Eigen::Vector3d const z = (Eigen::Vector3d() <<
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        tf::getYaw(msg.pose.pose.orientation)
    ).finished();

    Eigen::Map<Eigen::Matrix<double, 6, 6> const> cov_raw(
        &msg.pose.covariance.front()
    );

    Eigen::Matrix3d cov_z;
    cov_z.topLeftCorner<2, 2>() = cov_raw.topLeftCorner<2, 2>();
    cov_z(2, 2) = cov_raw(5, 5);

    // TODO: Why does this line cause a compile-time error?
    //kf.update_encoders(z, cov_z);
    if (watch_encoders) publish();
}

static void updateGps(nav_msgs::Odometry const &msg)
{
    // TODO: TF frame conversion
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_kf_node");

    ros::NodeHandle nh;
    sub_compass  = nh.subscribe("compass", 1, &updateCompass);
    sub_encoders = nh.subscribe("encoders", 1, &updateEncoders);
    sub_gps      = nh.subscribe("gps", 1, &updateGps);
    pub_fused    = nh.advertise<nav_msgs::Odometry>("fused", 100);

    ros::spin();
    return 0;
}
