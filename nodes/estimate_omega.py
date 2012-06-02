#!/usr/bin/env python
import roslib; roslib.load_manifest('robot_kf')
import rospy
import math
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu

prev_yaw = None
prev_time = None
samples = list()

def get_yaw(qt):
    from tf.transformations import euler_from_quaternion
    qt_array = np.array([ qt.x, qt.y, qt.z, qt.w ])
    roll, pitch, yaw = euler_from_quaternion(qt_array)
    return yaw

def callback(imu):
    global prev_yaw, prev_time

    yaw = get_yaw(imu.orientation)
    time = imu.header.stamp

    if prev_yaw != None:
        delta_raw = (yaw - prev_yaw) % (2 * math.pi)
        delta = min(delta_raw, 2 * math.pi - delta_raw)
        timestep = (time - prev_time).to_sec()
        omega = delta / timestep

        print omega
        samples.append(omega)

    prev_yaw = yaw
    prev_time = imu.header.stamp

def main():
    global pub_vel
    rospy.init_node('estimate_omega', anonymous=True)
    sub_gps = rospy.Subscriber('compass', Imu, callback)
    pub_vel = rospy.Publisher('omega', Float64)

    rospy.spin()

    print 'Average Angular Velocity =', sum(samples) / len(samples), 'rad/s'

if __name__ == '__main__':
    main()
