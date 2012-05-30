#!/usr/bin/env python
import roslib; roslib.load_manifest('robot_kf')
import rospy
import math
import numpy as np
import scipy.optimize
from nav_msgs.msg import Odometry
from robot_kf.msg import WheelOdometry
from sensor_msgs.msg import Imu

class OdometryCalibrator:
    def __init__(self):
        self.time_gps = list()
        self.time_compass = list()
        self.time_odom = list()
        self.data_gps = list()
        self.data_compass = list()
        self.data_odom = list()

    def setup(self, topic_odom, topic_gps, topic_compass):
        self.sub_odom = rospy.Subscriber(topic_odom, WheelOdometry, self.callback_odom)
        self.sub_gps = rospy.Subscriber(topic_gps, Odometry, self.callback_gps)
        self.sub_compass = rospy.Subscriber(topic_compass, Imu, self.callback_compass)

    def callback_gps(self, msg):
        datum_gps = self._get_odom_pose(msg)
        self.time_gps.append(msg.header.stamp)
        self.data_gps.append(datum_gps[0:2])

    def callback_compass(self, msg):
        datum_compass = self._get_compass_yaw(msg)
        self.time_compass.append(msg.header.stamp)
        self.data_compass.append(datum_compass)

    def callback_odom(self, msg):
        datum_odom = self._get_wheel_movement(msg)
        self.time_odom.append(msg.header.stamp)
        self.data_odom.append(datum_odom)

    def optimize(self, guess, alpha):
        gps = np.array(self.data_gps)
        compass = np.array(self.data_compass)
        odom = np.array(self.data_odom)

        # Find the distance between subsequent GPS samples.
        zero = np.zeros((1, 2))
        gps_delta = np.vstack((gps, zero)) - np.vstack((zero, gps))
        gps_linear = np.hypot(gps_delta[:, 0], gps_delta[:, 1])[1:-1]
        compass_delta = (np.vstack((compass, 0)) - np.vstack((0, compass)))[1:-1]

        # Truncate the first and last rows, since they are not valid deltas.
        self.time_gps.pop()
        self.time_compass.pop()

        def objective(params):
            rl, rr, s = params
            i_gps, i_compass = 0, 0
            i_odom_gps, i_odom_compass = 0, 0
            odom_gps, odom_compass = np.zeros(3), np.zeros(3)
            error_gps, error_compass = 0.0, 0.0

            for i_odom in xrange(1, odom.shape[0]):
                # Compare the relative movement estimated by the odometry with
                # that measured by the GPS.
                advance = 0
                while i_gps < len(self.time_gps) and self.time_gps[i_gps] < self.time_odom[i_odom]:
                    advance += 1
                    i_gps += 1

                if advance > 0:
                    # Correct for the delay between the last odom update and this GPS update.
                    if i_gps > 0:
                        dt_gps = self.time_gps[i_gps - 1] - self.time_gps[i_gps - 2]
                        dt_odom = self.time_odom[i_odom] - self.time_odom[i_odom_gps]
                        odom_gps *= dt_gps.to_sec() / dt_odom.to_sec()

                    error_gps += abs(np.linalg.norm(odom_gps[0:2]) - gps_linear[i_gps - 1])
                    odom_gps = np.zeros(3)
                    i_odom_gps = i_odom

                # Compare the change in heading with the compass measurements.
                advance = 0
                while i_compass < len(self.time_compass) and self.time_compass[i_compass] < self.time_odom[i_odom]:
                    advance += 1
                    i_compass += 1

                if advance > 0:
                    # Correct for the delay between the last odom update and this compass update.
                    if i_compass > 0:
                        dt_compass = self.time_compass[i_compass - 1] - self.time_compass[i_compass - 2]
                        dt_odom = self.time_odom[i_odom] - self.time_odom[i_odom_compass]
                        odom_compass *= dt_compass.to_sec() / dt_odom.to_sec()

                    error_compass += abs(odom_compass[2] - compass_delta[i_compass - 1, 0])
                    odom_compass = np.zeros(3)
                    i_odom_compass = i_odom

                # Integrate the wheel odometry to estimate the change in pose.
                linear  = (rr * odom[i_odom, 1] + rl * odom[i_odom, 0]) / 2
                angular = (rr * odom[i_odom, 1] - rl * odom[i_odom, 0]) * s
                odom_gps += np.array([ linear * math.cos(odom_gps[2] + angular / 2),
                                       linear * math.sin(odom_gps[2] + angular / 2),
                                       angular ])
                odom_compass += np.array([ linear * math.cos(odom_compass[2] + angular / 2),
                                           linear * math.sin(odom_compass[2] + angular / 2),
                                           angular ])

            error_gps /= len(gps_linear)
            error_compass /= len(compass_delta)
            return error_gps + alpha * error_compass

        # TODO: Restrict the parameters to be positive.
        params = scipy.optimize.fmin_slsqp(objective, guess, iprint=2)
        return np.array([ params[0], params[1], 1.0 / params[2] ])

    @classmethod
    def _get_odom_pose(cls, msg):
        position = msg.pose.pose.position
        yaw = cls._get_yaw(msg.pose.pose.orientation)
        return [ position.x, position.y, yaw ]

    @classmethod
    def _get_compass_yaw(cls, msg):
        yaw = cls._get_yaw(msg.orientation)
        return [ yaw ]

    @classmethod
    def _get_wheel_movement(cls, msg):
        return [ msg.left.movement, msg.right.movement ]

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
    guess_diameter = rospy.param('~diameter', 0.1)
    guess_separation = rospy.param('~separation', 1.0)
    weight = rospy.param('~weight', 5.0)

    guess = np.array([ diameter, diameter, 1.0 / guess_separation ])
    calibrator.optimize(guess, weight)

if __name__ == '__main__':
    main()

