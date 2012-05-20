#!/usr/bin/env python
import roslib; roslib.load_manifest('robot_kf')
import numpy as np, rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class OdometryCalibrator:
    def __init__(self):
        self.curr_odom = None
        self.prev_gps = None
        self.prev_compass = None
        self.prev_odom_gps = None
        self.prev_odom_compass = None
        self.data_gps = list()
        self.data_compass = list()

    def setup(self, topic_odom, topic_gps, topic_compass):
        self.sub_odom = rospy.Subscriber(topic_odom, Odometry, self.callback_odom)
        self.sub_gps = rospy.Subscriber(topic_gps, Odometry, self.callback_gps)
        self.sub_compass = rospy.Subscriber(topic_compass, Imu, self.callback_compass)

    def callback_gps(self, curr_gps):
        if not (self.prev_gps and self.curr_odom and self.prev_odom_gps):
            self.prev_gps = curr_gps
            return 

        delta_gps = self._get_state(curr_pos) - self._get_state(prev_pos)
        delta_gps_time = curr_gps.header.stamp - self.prev_gps.header.stamp

        delta_odom = self._get_state(curr_odom_pos) - self._get_state(prev_odom_pos)
        delta_odom_time = curr_odom.header.stamp - self.prev_odom_gps.header.stamp

        # Correct of the time delay between the last odometry update and
        # the current GPS update. This difference is always non-negative.
        delta_odom *= delta_gps_time / delta_odom_time

        self.data_gps.append([ delta_gps[0:2], delta_odom[0:2] ])
        self.prev_odom_gps = self.curr_odom
        self.prev_gps = curr_gps

    def callback_compass(self, curr_compass):
        if not (self.prev_compass and self.curr_odom and self.prev_odom_compass):
            self.prev_compass = curr_compass
            return

        delta_compass = self._get_state(curr_compass) - self._get_state(self.prev_compass)
        delta_compass_time = curr_compass.header.stamp - self.prev_compass.header.stamp

        delta_odom = self._get_state(curr_odom_pos) - self._get_state(prev_odom_pos)
        delta_odom_time = curr_odom.header.stamp - self.prev_odom_gps.header.stamp

        # Correct for the time delay. See callback_gps() for details.
        delta_odom *= delta_compass_time / delta_odom_time

        self.data_compass.append([ delta_compass[2], delta_odom[2] ])
        self.prev_odom_compass = self.curr_odom
        self.prev_compass = curr_compass

    def callback_odom(self, curr_odom):
        self.curr_odom = curr_odom

    @staticmethod
    def _get_state(msg):
        from tf.transformations import euler_from_quaternion
        if isinstance(msg, Odometry):
            position = msg.pose.pose.position
            roll, pitch, yaw = euler_from_quaternion(msg.pose.pose.orientation)
            return np.array([ position.x, position.y, yaw ])
        elif isinstance(msg, Imu):
            roll, pitch, yaw = euler_from_quaternion(msg.orientation)
            return np.array([ 0, 0, yaw ])
        else:
            return None

def main():
    rospy.init_node('diffdrive_calibrator', anonymous=True)
    calibrator = Calibrator()
    calibrator.setup('odom', 'gps', 'compass')
    rospy.spin()

if __name__ == '__main__':
    main()

