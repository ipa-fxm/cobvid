#!/usr/bin/env python2
#-*- coding: utf-8 -*-

import rospy
import yaml
import os
import errno

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

        self.last_data_left = None
        self.last_data_right = None

        self.position_data = {'left': list(), 'right': list()}


    def initRos(self):
        rospy.init_node('goal_state_recorder')

        if self.mode & GSRecorder.MODE_LEFT:
            self.subl = rospy.Subscriber(GSRecorder.TOPIC_LEFT, JointTrajectoryControllerState, self.left_callback)

        if self.mode & GSRecorder.MODE_RIGHT:
            self.subr = rospy.Subscriber(GSRecorder.TOPIC_RIGHT, JointTrajectoryControllerState, self.right_callback)


    def left_callback(self, data):
        self.last_data_left = data.actual.positions

    def right_callback(self, data):
        self.last_data_right = data.actual.positions

    def grep_data(self, data):
        self.position_data['left'].append(self.last_data_left)
        self.position_data['right'].append(self.last_data_right)

    def record(self):
        self.initRos()
        #self.timer.start()

        self.timer = rospy.Timer(rospy.Duration(1.0), self.grep_data)
        rospy.spin()
        #self.timer.join(1)

        print 'ENDING'

        print self.mode

    def save(self):
        try:
            os.makedirs('../trajectory_goal_data')
        except OSError as ex:
            if ex.errno != errno.EEXIST:
                raise

        with open('../trajectory_goal_data/trajectory_goal.yaml', 'w') as f:
            yaml.safe_dump(self.position_data, f)


if __name__ == '__main__':
    gsr = GSRecorder()
    raw_input('press enter to record trajectory_goal_data')
    gsr.record()
    rospy.spin()
    print 'recording stopped'
    print 'save recorded data...'
    gsr.save()

