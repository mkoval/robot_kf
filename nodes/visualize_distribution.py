#!/usr/bin/env python
from __future__ import print_function
import roslib; roslib.load_manifest('robot_kf')
import rospy
import math
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import create_cloud, PointCloud

num_samples = 100

def callback(odom):
    position = odom.pose.pose.position

    mean = np.array([ position.x, position.y, position.z ])
    cov = np.array(odom.pose.covariance)
    np.reshape(cov, (6, 6))
    samples = np.random.multivariate_normal(mean, cov, num_samples)

    print samples

def main():
    global pub_cloud

    rospy.init_node('visualize_distribution', anonymous=True)
    sub_odom  = rospy.Subscriber('odom_fused', Odometry, callback)
    pub_cloud = rospy.Publisher('odom_distribution', PointCloud2)

    rospy.spin()

if __name__ == '__main__':
    main()
