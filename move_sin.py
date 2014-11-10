#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import numpy as np
import time


x = np.linspace(0, np.pi * 2, 200)
xsin = np.sin(x)
maxvel = 0.1
xsin *= maxvel

xTwist = Twist()

rospy.init_node('VID_TEST')
pub = rospy.Publisher('/base_controller/command', Twist)

for vel in xsin:
    xTwist.linear.x = vel
    pub.publish(xTwist)
    time.sleep(0.01)

'''
stopTwist = Twist()
print stopTwist
print xTwist

rospy.init_node('VID_TEST')
pub = rospy.Publisher('/base_controller/command', Twist)


time.sleep(1)
pub.publish(xTwist)
time.sleep(1)
pub.publish(stopTwist)

'''
#while not rospy.is_shutdown():
#    pass