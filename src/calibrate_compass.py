#!/usr/bin/env python
from __future__ import print_function
import roslib; roslib.load_manifest('robot_kf')
import rospy
import curses
import math
import numpy as np
import scipy.optimize
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

collecting = False

class CompassCalibrator:
    def setup(self, topic_gps, topic_compass):
        self.sub_gps = rospy.Subscriber(topic_gps, Odometry, self.callback_gps)
        self.sub_compass = rospy.Subscriber(topic_compass, Imu, self.callback_compass)

    def callback_gps(self, msg):
        


def main(screen):
    curses.cbreak()
    curses.noecho()

    screen.addstr(0, 0, 'Data Collection: Press <j> to resume, <k> to pause, or <e> to end.')
    screen.addstr(1, 0, '[PAUSED] Samples Collected: 0')
    screen.refresh()

    while True:
        ch = chr(screen.getch())

        if ch == 'j':
            collecting = True
            screen.deleteln()
            screen.addstr(1, 0, '[RUNNING] Samples Collected: 0')
        elif ch == 'k':
            collecting = False
            screen.deleteln()
            screen.addstr(1, 0, '[PAUSED] Samples Collected: 0')
        elif ch == 'q':
            break

            

if __name__ == '__main__':
    curses.wrapper(main)

