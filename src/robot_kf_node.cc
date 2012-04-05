#include <Eigen/Geometry>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <robot_kf/robot_kf.h>

static boost::shared_ptr<tf::TransformListener> sub_tf;
static ros::Subscriber sub_compass;
static ros::Subscriber sub_encoders;
static ros::Subscriber sub_gps;
static ros::Publisher  pub_fused;
static robot_kf::KalmanFilter kf;
static std::string frame;

static bool watch_compass;
static bool watch_encoders;
static bool watch_gps;
static std::string frame_id;

static void publish(void)
{
}

static void updateCompass(sensor_msgs::Imu const &msg)
{
    // FIXME: Also transform the covariance matrix.
    geometry_msgs::QuaternionStamped q_src, q_dst;
    q_src.header = msg.header;
    q_src.quaternion = msg.orientation;
    sub_tf->transformQuaternion(frame_id, q_src, q_dst);

    double const yaw = tf::getYaw(q_dst.quaternion);
    double const cov = msg.orientation_covariance[8];

    kf.update_compass(yaw, cov);
    if (watch_compass) publish();
}

static void updateEncoders(nav_msgs::Odometry const &msg)
{
    // FIXME: Also transform the covariance matrix.
    geometry_msgs::PoseStamped p_src, p_dst;
    p_src.header = msg.header;
    p_src.pose = msg.pose.pose;
    sub_tf->transformPose(frame_id, p_src, p_dst);

    Eigen::Vector3d const z = (Eigen::Vector3d() <<
        p_src.pose.position.x,
        p_src.pose.position.y,
        tf::getYaw(msg.pose.pose.orientation)
    ).finished();

    Eigen::Map<Eigen::Matrix<double, 6, 6> const> cov_raw(
        &msg.pose.covariance.front()
    );

    Eigen::Matrix3d cov_z = Eigen::Matrix3d::Zero();
    cov_z.topLeftCorner<2, 2>() = cov_raw.topLeftCorner<2, 2>();
    cov_z(2, 2) = cov_raw(5, 5);

    // TODO: Why does this line cause a compile-time error?
    kf.update_encoders(z, cov_z);
    if (watch_encoders) publish();
}

static void updateGps(nav_msgs::Odometry const &msg)
{
    // FIXME: Also transform the covariance matrix.
    geometry_msgs::PoseStamped p_src, p_dst;
    p_src.header = msg.header;
    p_src.pose = msg.pose.pose;
    sub_tf->transformPose(frame_id, p_src, p_dst);

    Eigen::Vector2d const z = (Eigen::Vector2d() <<
        p_dst.pose.position.x,
        p_dst.pose.position.y
    ).finished();

    Eigen::Map<Eigen::Matrix<double, 6, 6> const> cov_raw(
        &msg.pose.covariance.front()
    );
    Eigen::Matrix2d const cov_z = cov_raw.topLeftCorner<2, 2>();

    kf.update_gps(z, cov_z);
    if (watch_gps) publish();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_kf_node");

    ros::NodeHandle nh;
    nh.param<bool>("watch_compass",  watch_compass,  true);
    nh.param<bool>("watch_encoders", watch_encoders, true);
    nh.param<bool>("watch_gps",      watch_gps,      true);
    nh.param<std::string>("frame_id", frame_id, "/odom_fused");

    sub_tf = boost::make_shared<tf::TransformListener>();
    sub_compass  = nh.subscribe("compass", 1, &updateCompass);
    sub_encoders = nh.subscribe("odom", 1, &updateEncoders);
    sub_gps      = nh.subscribe("gps", 1, &updateGps);
    pub_fused    = nh.advertise<nav_msgs::Odometry>("odom_fused", 100);

    ros::spin();
    return 0;
}
