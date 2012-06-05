#!/usr/bin/env python
import roslib; roslib.load_manifest('robot_kf')
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

last_vel   = 0.0
last_omega = 0.0
last_stamp = rospy.Time(0)

threshold_accel = 0.1
threshold_alpha = 0.1

ignore_start = rospy.Time(0)
ignore_duration = rospy.Duration(0.25)

def callback_odom(msg):
    global last_vel, last_omega, last_stamp, ignore_start

    curr_vel = msg.twist.twist.linear.x
    curr_omega = msg.twist.twist.angular.z
    dt = (msg.header.stamp - last_stamp).to_sec()

    accel = (curr_vel - last_vel) / dt
    alpha = (curr_omega - last_omega) / dt

    #rospy.loginfo('a = {0}, alpha = {1}'.format(accel, alpha))

    if abs(accel) > threshold_accel or abs(alpha) > threshold_alpha:
        rospy.logwarn('Ignoring compass during acceleration.')
        ignore_start = msg.header.stamp

    last_vel = curr_vel
    last_omega = curr_omega
    last_stamp = msg.header.stamp

def callback_compass(msg):
    if msg.header.stamp - ignore_start > ignore_duration:
        pub_filtered.publish(msg)

def main():
    global threshold_accel, threshold_alpha, pub_filtered

    rospy.init_node('compass_filter')

    threshold_accel = rospy.get_param('~threshold_accel', 1.0)
    threshold_alpha = rospy.get_param('~threshold_alpha', 1.0)

    sub_twist = rospy.Subscriber('drive/odom', Odometry, callback_odom)
    sub_compass = rospy.Subscriber('compass', Imu, callback_compass)
    pub_filtered = rospy.Publisher('compass_filtered', Imu)

    rospy.spin()

if __name__ == '__main__':
    main()

