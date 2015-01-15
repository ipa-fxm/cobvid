#!/usr/bin/env python2
#-*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from control_msgs.msg import JointTrajectoryControllerState

class GSRecorder(object):
    MODE_LEFT = 0b1
    MODE_RIGHT = 0b10
    MODE_BOTH = MODE_LEFT | MODE_RIGHT

    TOPIC_LEFT = '/arm_left/joint_trajectory_controller/state'
    TOPIC_RIGHT = '/arm_right/joint_trajectory_controller/state'

    def __init__(self, mode=MODE_BOTH):
        self.mode = mode

    def initRos(self):
        rospy.init_node('goal_state_recorder')

        if self.mode & GSRecorder.MODE_LEFT:
            self.subl = rospy.Subscriber(GSRecorder.TOPIC_LEFT, JointTrajectoryControllerState, self.left_callback)

        if self.mode & GSRecorder.MODE_RIGHT:
            self.subr = rospy.Subscriber(GSRecorder.TOPIC_RIGHT, JointTrajectoryControllerState, self.right_callback)

    def left_callback(self, data):
        print 'left'
        print data
        print '-'*100

    def right_callback(self, data):
        print 'right'
        print data
        print '-'*100

    def record(self):
        self.initRos()
        print self.mode



if __name__ == '__main__':
    gsr = GSRecorder()
    gsr.record()
    rospy.spin()
    print 'end'