#!/usr/bin/env python
from __future__ import print_function
import roslib; roslib.load_manifest('robot_kf')
import rospy
import math
import numpy as np
import scipy.optimize
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class CompassCalibrator:
    def __init__(self):
        self.collecting = False
        self.data_gps = list()
        self.data_compass = list()

    def setup(self, topic_gps, topic_compass):
        self.sub_gps = rospy.Subscriber(topic_gps, Odometry, self.callback_gps)
        self.sub_compass = rospy.Subscriber(topic_compass, Imu, self.callback_compass)

    def start(self):
        self.collecting = True

    def stop(self):
        self.collecting = False

    def estimate_offset(self):
        if len(self.data_gps) < 2:
            rospy.logerr('Calibration requires a minimum of two GPS samples.')
            return None
        elif len(self.data_compass) < 1:
            rospy.logerr('Calibration requires a minimum of one compass sample.')
            return None

        gps_delta =  self.data_gps[-1] - self.data_gps[0]
        gps_angle = math.atan2(gps_delta[1], gps_delta[0])
        compass_angle = sum(self.data_compass) / len(self.data_compass)
        return compass_angle - gps_angle

    def callback_gps(self, msg):
        if self.collecting:
            position = self._get_gps_pos(msg)
            self.data_gps.append(position)

    def callback_compass(self, msg):
        if self.collecting:
            yaw = self._get_compass_yaw(msg)
            self.data_compass.append(yaw)
        
    @classmethod
    def _get_gps_pos(cls, msg):
        position = msg.pose.pose.position
        return np.array([ position.x, position.y ])

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
    rospy.init_node('calibrate_compass', anonymous=True)
    calibrator = CompassCalibrator()
    calibrator.setup('gps', 'compass')

    calibrator.start()
    rospy.spin()
    calibrator.stop()

    offset = calibrator.estimate_offset()
    rospy.loginfo('Computed offset using {0} samples.'.format(len(calibrator.data_compass)))
    rospy.loginfo('Offset to subtract from compass heading = {0} rads = {1} degs'.format(
        offset, math.degrees(offset)
    ))

if __name__ == '__main__':
    main()
