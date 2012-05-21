#!/usr/bin/env python
import roslib; roslib.load_manifest('robot_kf')
import numpy as np, rospy
from nav_msgs.msg import Odometry
from robot_kf.msg import WheelOdometry
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
        self.data_odom = list()

    def setup(self, topic_odom, topic_gps, topic_compass):
        self.sub_odom = rospy.Subscriber(topic_odom, WheelOdometry, self.callback_odom)
        self.sub_gps = rospy.Subscriber(topic_gps, Odometry, self.callback_gps)
        self.sub_compass = rospy.Subscriber(topic_compass, Imu, self.callback_compass)

    def callback_gps(self, curr_gps):
        if not (self.prev_gps and self.curr_odom and self.prev_odom_gps):
            self.prev_gps = curr_gps
            return 

        delta_gps = self._get_state(curr_gps) - self._get_state(self.prev_gps)
        delta_gps_time = curr_gps.header.stamp - self.prev_gps.header.stamp

        self.data_gps.append(list(delta_gps[0:2]))
        self.prev_gps = curr_gps

    def callback_compass(self, curr_compass):
        if not (self.prev_compass and self.curr_odom and self.prev_odom_compass):
            self.prev_compass = curr_compass
            return

        delta_compass = self._get_state(curr_compass) - self._get_state(self.prev_compass)
        delta_compass_time = curr_compass.header.stamp - self.prev_compass.header.stamp

        self.data_compass.append(delta_compass[2])
        self.prev_odom_compass = self.curr_odom
        self.prev_compass = curr_compass

    def callback_odom(self, curr_odom):
        self.prev_odom = self.curr_odom
        self.curr_odom = curr_odom
        self.data_odom.append(list(self._get_state(curr_odom)))

        if not self.prev_odom_gps:
            self.prev_odom_gps = curr_odom

        if not self.prev_odom_compass:
            self.prev_odom_compass = curr_odom

    def optimize(self):
        x_gps = np.array(self.data_gps)
        x_compass = np.array(self.data_compass)
        x_odom = np.array(self.data_odom)

        print 'gps = ', x_gps
        print 'compass = ', x_compass
        print 'odom = ', x_odom

    @classmethod
    def _get_state(cls, msg):
        if isinstance(msg, Odometry):
            position = msg.pose.pose.position
            yaw = cls._get_yaw(msg.pose.pose.orientation)
            return np.array([ position.x, position.y, yaw ])
        elif isinstance(msg, WheelOdometry):
            return np.array([ msg.left.movement, msg.right.movement ])
        elif isinstance(msg, Imu):
            yaw = cls._get_yaw(msg.orientation)
            return np.array([ 0, 0, yaw ])
        else:
            return None

    @classmethod
    def _get_yaw(cls, qt):
        from tf.transformations import euler_from_quaternion
        qt_array = np.array([ qt.x, qt.y, qt.z, qt.w ])
        roll, pitch, yaw = euler_from_quaternion(qt_array)
        return yaw
        

def main():
    rospy.init_node('diffdrive_calibrator', anonymous=True)
    calibrator = OdometryCalibrator()
    calibrator.setup('wheel_odom', 'gps', 'compass')

    rospy.spin()

    calibrator.optimize()

if __name__ == '__main__':
    main()

