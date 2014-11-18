#!/usr/bin/env python2
#try:
import roslib
try:
    isLive=True
    roslib.load_manifest('cobvid')
except:
    isLive=False
    print
    print '#'*39
    print '# NO ROS ENVIRONMENT - SKIP EXECUTION #'
    print '#'*39
    print

import rosnode
import rospy
import actionlib
import control_msgs.msg
import trajectory_msgs.msg

from actionlib import simple_action_client
from control_msgs.msg import JointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryAction

from trajectory_msgs.msg import JointTrajectoryPoint
import numpy as np
from itertools import repeat
import math as m

class JTP(object):

    def __init__(self, rel_time=0,
                 p1=None, p2=None, p3=None, p4=None, p5=None, p6=None, p7=None,
                 v1=0, v2=0, v3=0, v4=0, v5=0, v6=0, v7=0, jtp=None):

        positions = [p1, p2, p3, p4, p5, p6, p7]
        for k, v in enumerate(positions):
            positions[k] = v if v else jtp.point.positions[k] if jtp else 0


        self.point = trajectory_msgs.msg.JointTrajectoryPoint()

        if isinstance(jtp, JTP):
            self.point.positions = jtp.point.positions


        self.point.positions = positions
        velocities = [v1, v2, v3, v4, v5, v6, v7]
        if np.abs(np.array(velocities)).sum() != 0:
            self.point.velocities = velocities
        self.time_from_start = rel_time

    @staticmethod
    def get_mirrored_jtp_list(jtp_list):
        new_list = list()
        for jtp in jtp_list:
            new_list.append(JTP.get_mirrored_jtp(jtp))
        return new_list

    @staticmethod
    def get_mirrored_jtp(jtp):
        positions = [p*-1 for p in jtp.point.positions]
        return JTP(jtp.time_from_start, *positions)


    @staticmethod
    def _updateTimes(jtp_list):
        cur_time = 0
        for jtp in jtp_list:
            cur_time += jtp.time_from_start
            jtp.point.time_from_start = rospy.Duration(cur_time)

    @staticmethod
    def get_point_list(jtp_list):
        JTP._updateTimes(jtp_list)
        return [jtp.point for jtp in jtp_list]

    @staticmethod
    def get_total_time_duration(*jtp_lists):
        total_time = list()
        for idx, jtp_list in enumerate(jtp_lists):
            list_time = sum([jtp.time_from_start for jtp in jtp_list])
            total_time.append(list_time)
        return max(total_time)

    def __repr__(self):
        return 'TimeFromstart: %s - Positions: %s - Velocities: %s' % \
               (self.point.time_from_start, [m.degrees(p) for p in self.point.positions], self.point.velocities)

class SynchronousTrajectory():
    def __init__(self, ros_rate=10):
        NODENAME = 'arm_traj_action'
        rospy.init_node(NODENAME)
        r = rospy.Rate(ros_rate)

        for i in range(30):
            if '/' + NODENAME in rosnode.get_node_names():
                break
            print 'establishing node...'
            rospy.sleep(0.1)
        else:
            print 'timout reached, node creation failed. exiting application'
            exit()

        self.action_client_right = actionlib.SimpleActionClient('/arm_right/joint_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.action_client_left = actionlib.SimpleActionClient('/arm_left/joint_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.action_client_right.wait_for_server()
        self.action_client_left.wait_for_server()

    def get_joint_names(self, is_left_arm=True):
        joint_name_pattern = ['arm_%s_%d_joint']*7
        direction = 'left' if is_left_arm else 'right'
        return [pattern % (direction, idx+1) for idx, pattern in enumerate(joint_name_pattern)]

    def send_jtp_list_synchronous(self, jtp_list_left, jtp_list_right):
        self.send_jtp_list(jtp_list_left, is_left_arm=True)
        self.send_jtp_list(jtp_list_right, is_left_arm=False)
        return JTP.get_total_time_duration(jtp_list_left, jtp_list_right)

    def send_jtp_list(self, jtp_list, is_left_arm=True):
        print 'Sending following JTP-Points to %s arm:' % ('left' if is_left_arm else 'right')
        for jtp in jtp_list:
            print jtp
        action_client = self.action_client_left if is_left_arm else self.action_client_right
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.get_joint_names(is_left_arm)
        goal.trajectory.points = JTP.get_point_list(jtp_list)
        action_client.send_goal(goal)
        rospy.sleep(0.05)
        return JTP.get_total_time_duration(jtp_list)

class ArmMovement(object):

    def home_position(self):
        return [JTP(10)], [JTP(10)]


    def boring_movement_variant(self):

        back_left =      {'p1': m.radians(35),  'p2': m.radians(-65), 'p3':  m.radians(89),  'p4': m.radians(60)}
        norm_left =      {'p1': m.radians(55),  'p2': m.radians(-58), 'p3':  m.radians(90),  'p4': m.radians(70)}
        norm_front_cp1 = {'p1': m.radians(85),  'p2': m.radians(-54), 'p3': m.radians(130),  'p4': m.radians(74)}
        front_left =     {'p1': m.radians(120), 'p2': m.radians(-65), 'p3': m.radians(160),  'p4': m.radians(75)}

        time_delta = 4
        jtp_list_left = list()
        #jtp_list_left.append(JTP(rel_time=8))
        jtp_list_left.append(JTP(rel_time=4, **norm_left))
        for i in range(1):
            jtp_list_left.append(JTP(rel_time=time_delta, **back_left))
            jtp_list_left.append(JTP(rel_time=time_delta, **norm_left))
            jtp_list_left.append(JTP(rel_time=time_delta, **norm_front_cp1))
            jtp_list_left.append(JTP(rel_time=time_delta, **front_left))
            jtp_list_left.append(JTP(rel_time=time_delta, **norm_front_cp1))
            jtp_list_left.append(JTP(rel_time=time_delta, **norm_left))

        return jtp_list_left, list()

    def boring_movement(self):

        norm_left = {'p1': m.radians(55), 'p2': m.radians(-60), 'p3': m.radians(90), 'p4': m.radians(70)}
        front_left = {'p1': m.radians(100), 'p2': m.radians(-60), 'p3': m.radians(130), 'p4': m.radians(90)}
        front_boring_left = {'p1': m.radians(80), 'p2': m.radians(-60), 'p3': m.radians(110), 'p4': m.radians(80)}

        time_delta = 3
        jtp_list_left = list()

        # go to normal position - long duration because it's unclear where was the last position
        jtp_list_left.append(JTP(rel_time=4, **norm_left))

        # build movement seperate so split-time movment is the reversed mirrored list
        jtp_list_move_left = list()
        for i in range(4):
            jtp_list_move_left.append(JTP(rel_time=time_delta, **norm_left))
            jtp_list_move_left.append(JTP(rel_time=time_delta, **front_boring_left))

        # extend lists for both sides with mirrored and reversed movement list
        jtp_list_right = JTP.get_mirrored_jtp_list(jtp_list_left)
        jtp_list_move_right = JTP.get_mirrored_jtp_list(jtp_list_move_left[::-1])

        jtp_list_left.extend(jtp_list_move_left)
        jtp_list_right.extend(jtp_list_move_right)

        # go to normal position on both arms
        jtp_list_left.append(JTP(rel_time=time_delta, **norm_left))
        jtp_list_right.append(JTP.get_mirrored_jtp(JTP(rel_time=time_delta, **norm_left)))

        return jtp_list_left, jtp_list_right

    def cross_arms_behind(self):
        norm_left = {'p1': m.radians(55), 'p2': m.radians(-60), 'p3': m.radians(90), 'p4': m.radians(70)}

        jtp_list_left = list()
        jtp_list_left.append(JTP(rel_time=4, **norm_left))
        jtp_list_right = JTP.get_mirrored_jtp_list(jtp_list_left)

        return jtp_list_left, jtp_list_right

if __name__=='__main__':

    if isLive:
        st = SynchronousTrajectory()
        am = ArmMovement()

        #rospy.sleep(st.send_jtp_list_synchronous(*am.home_position()))

        #rospy.sleep(st.send_jtp_list_synchronous(*am.boring_movement_variant()))
        #rospy.sleep(st.send_jtp_list_synchronous(*am.boring_movement()))

        rospy.sleep(st.send_jtp_list_synchronous(*am.cross_arms_behind()))


        print
        print '*%s*' % ('-'*17)
        print '| SCRIPT FINISHED |'
        print '*%s*' % ('-'*17)
        print


