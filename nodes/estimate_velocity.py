#!/usr/bin/env python
from __future__ import print_function
import roslib; roslib.load_manifest('robot_kf')
import rospy
import math
import numpy as np
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

prev_pos = None
prev_time = None
samples = list()

def callback(odom):
    global prev_pos, prev_time

    pos = np.array([ odom.pose.pose.position.x, odom.pose.pose.position.y ])
    time = odom.header.stamp

    if prev_pos:
        distance = np.linalg.norm(pos - prev_pos, 2)
        timestep = (time - prev_time).to_sec()
        velocity = distance / timestep

        samples.append(velocity)
        pub_vel.publish(velocity)

    prev_pos = pos
    prev_time = odom.header.stamp

def main():
    global pub_vel
    rospy.init_node('estimate_velocity', anonymous=True)
    sub_gps = rospy.Subscriber('gps', Odometry, callback)
    pub_vel = rospy.Publisher('velocity', Float64)

    rospy.spin()

    print 'Average Velocity =', sum(samples) / len(samples), 'm/s'

if __name__ == '__main__':
    main()
