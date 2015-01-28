#!/usr/bin/env python2
#-*- coding: utf-8 -*-

import rospy
import yaml
import os
import errno

from sensor_msgs.msg import JointState


class GSRecorder(object):

    def __init__(self):
        self.last_data_left = None
        self.last_data_right = None

        self.joint_data = {'left': list(), 'right': list()}

    def initRos(self):
        rospy.init_node('goal_state_recorder')
        rospy.Subscriber('/joint_states', JointState, self.joint_callback)

    def joint_callback(self, data):

        if 'arm_left_1_joint' in data.name:
            self.last_data_left = list(data.position)
            self.last_data_left.extend(data.velocity)

        if 'arm_right_1_joint' in data.name:
            self.last_data_right = list(data.position)
            self.last_data_right.extend(data.velocity)


    def grep_data(self, data):
        if self.last_data_left is not None and self.last_data_right is not None:
            self.joint_data['left'].append(self.last_data_left)
            self.joint_data['right'].append(self.last_data_right)
        else:
            print 'NOT RECORDED, MISSING DATA...'

        print 'left: ', self.last_data_left
        print 'right:', self.last_data_right
        print '--'


    def record(self):
        self.initRos()
        self.timer = rospy.Timer(rospy.Duration(1.0), self.grep_data)
        rospy.spin()

        print 'ENDING'

    def save(self):
        try:
            os.makedirs('../trajectory_goal_data')
        except OSError as ex:
            if ex.errno != errno.EEXIST:
                raise

        with open('../trajectory_goal_data/trajectory_goal.yaml', 'w') as f:
            yaml.safe_dump(self.joint_data, f)


if __name__ == '__main__':
    gsr = GSRecorder()

    raw_input('press enter to record trajectory_goal_data')
    print 'now recording...'

    gsr.record()
    rospy.spin()

    print 'recording stopped'
    print 'save recorded data...'

    gsr.save()

