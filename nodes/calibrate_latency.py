#!/usr/bin/env python
from __future__ import print_function
import roslib; roslib.load_manifest('robot_kf')
import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class CompassCalibrator:
    def __init__(self, target):
        self.collecting = False
        self.end_compass = None
        self.end_odom = None
        self.initial_compass = None
        self.initial_odom = None
        self.target = target
        self.ready = 0

    def setup(self, topic_odom, topic_compass):
        self.sub_odom = rospy.Subscriber(topic_odom, Odometry, self.callback_odom)
        self.sub_compass = rospy.Subscriber(topic_compass, Imu, self.callback_compass)

    def estimate_latency(self):
        return (self.end_compass - self.end_odom).to_sec()

    def callback_odom(self, msg):
        curr_odom = self._get_odom_yaw(msg)

        if self.initial_odom:
            delta = self._get_delta_angle(curr_odom, self.initial_odom)
            if self.ready == 2 and abs(delta) > self.target and not self.end_odom:
                self.end_odom = msg.header.stamp
                rospy.loginfo("Done collecting odometry samples.")
        elif not self.initial_odom:
            rospy.loginfo('Initialized odometry.')
            self.initial_odom = curr_odom
            self.ready += 1


    def callback_compass(self, msg):
        curr_compass = self._get_compass_yaw(msg)

        if self.initial_compass:
            delta = self._get_delta_angle(curr_compass, self.initial_compass)
            if self.ready == 2 and abs(delta) > self.target and not self.end_compass:
                self.end_compass = msg.header.stamp
                rospy.loginfo("Done collecting compassetry samples.")
        else:
            rospy.loginfo('Initialized compass.')
            self.initial_compass = curr_compass
            self.ready += 1

    @classmethod
    def _get_delta_angle(cls, a, b):
        raw = (b - a) % (2 * math.pi)
        return min(raw, 2 * math.pi - raw)
        
    @classmethod
    def _get_odom_yaw(cls, msg):
        return cls._get_yaw(msg.pose.pose.orientation)

    @classmethod
    def _get_compass_yaw(cls, msg):
        yaw = cls._get_yaw(msg.orientation)
        return yaw

    @classmethod
    def _get_yaw(cls, qt):
        from tf.transformations import euler_from_quaternion
        qt_array = np.array([ qt.x, qt.y, qt.z, qt.w ])
        roll, pitch, yaw = euler_from_quaternion(qt_array)
        return yaw

def main():
    rospy.init_node('calibrate_latency', anonymous=True)
    calibrator = CompassCalibrator(math.pi / 4)
    calibrator.setup('drive/odom', 'compass')

    rospy.spin()

    offset = calibrator.estimate_latency()
    rospy.loginfo('Estimate Latency = {0} s'.format(offset))

if __name__ == '__main__':
    main()
