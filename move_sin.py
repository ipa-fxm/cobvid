#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import numpy as np
import time


dofake = False

class Dummy(object):
    pass

def pub_fake(msg):
    print msg

x = np.linspace(0, np.pi * 2, 600)
xsin = np.sin(x)
maxvel = 0.2
xsin *= maxvel

xTwist = Twist()

if not dofake:
    rospy.init_node('VID_TEST')
    pub = rospy.Publisher('/base_controller/command', Twist)
else:
    pub = Dummy()
    pub.publish = pub_fake

for i in range(2):
    for vel in xsin:
        xTwist.linear.x = vel
        pub.publish(xTwist)
        time.sleep(0.01)