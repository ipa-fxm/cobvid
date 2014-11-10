#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import numpy as np
import time

stopTwist = Twist()
print stopTwist
xTwist = Twist()
xTwist.linear.x = 0.1
print xTwist

rospy.init_node('VID_TEST')
pub = rospy.Publisher('/base_controller/command', Twist)

time.sleep(1)
pub.publish(xTwist)
time.sleep(1)
pub.publish(stopTwist)


#while not rospy.is_shutdown():
#    pass