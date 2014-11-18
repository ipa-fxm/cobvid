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
        isPosSet = bool(sum([int(p is not None) for p in positions]))

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

    @staticmethod
    def extend_base_list(base_list=None, *jtp_lists):
        if not base_list:
            base_list = list([None]*len(jtp_lists))

        for idx, jtp_lists in enumerate(jtp_lists):
            if not base_list[idx]:
                base_list[idx] = list()
            base_list[idx].extend(jtp_lists)
        return base_list

    def __repr__(self):
        return 'TimeFromstart: %s - Positions: %s - Velocities: %s' % \
               (self.point.time_from_start, self.point.positions, self.point.velocities)
               #(self.point.time_from_start, [m.degrees(p) for p in self.point.positions], self.point.velocities)

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
    def __init__(self):
        self.pose_home = {'p1': 0, 'p2': 0, 'p3': 0, 'p4': 0, 'p5': 0, 'p6': 0, 'p7': 0}

        self.pose_cross_arms_behind = {'p1': m.radians(40), 'p2': m.radians(-65),
                                       'p3': m.radians(170), 'p4': m.radians(70)}

        self.pose_relaxed_arms_side = {'p1': m.radians(55), 'p2': m.radians(-60+30),
                                       'p3': m.radians(90), 'p4': m.radians(70)}

        self.pose_waiting_arms_side = {'p1': m.radians(50), 'p2': m.radians(-30),
                                       'p3': m.radians(120), 'p4': m.radians(90),
                                       'p5': m.radians(20), 'p6': m.radians(50)}

        self.pose_boring_walk_front = {'p1': m.radians(80), 'p2': m.radians(-60),
                                       'p3': m.radians(110), 'p4': m.radians(80)}

        self.pose_marsh_walk_front = {'p1': m.radians(100), 'p2': m.radians(-60+30),
                                      'p3': m.radians(130), 'p4': m.radians(90)}

    def movePose(self, pose, duration=4):
        jtp_list_left = list()

        jtp_list_left.append(JTP(rel_time=duration, **pose))
        jtp_list_right = JTP.get_mirrored_jtp_list(jtp_list_left)

        return jtp_list_left, jtp_list_right


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

    def movePoseSplit(self, pose1, pose2, duration=4, times=1):
        jtp_list_left = list()

        for i in range(times):
            jtp_list_left.append(JTP(rel_time=duration, **pose1))
            jtp_list_left.append(JTP(rel_time=duration, **pose2))

        jtp_list_right = JTP.get_mirrored_jtp_list(jtp_list_left[::-1])

        return jtp_list_left, jtp_list_right


if __name__=='__main__':

    if isLive:
        st = SynchronousTrajectory()
        am = ArmMovement()

        jtp_flow_data = list([None, None])
        #jtp_flow_data = JTP.extend_base_list(jtp_flow_data, *am.home_position())
        #jtp_flow_data = JTP.extend_base_list(jtp_flow_data, *am.cross_arms_behind(6))
        #jtp_flow_data = JTP.extend_base_list(jtp_flow_data, *am.home_position())



        #JTP.extend_base_list(jtp_flow_data, *am.movePose(am.pose_home, 6))
        #JTP.extend_base_list(jtp_flow_data, *am.movePose(am.pose_relaxed_arms_side, 6))

        #JTP.extend_base_list(jtp_flow_data, *am.movePose(am.pose_waiting_arms_side, 8))
        #JTP.extend_base_list(jtp_flow_data, *am.movePose(pose=am.pose_cross_arms_behind, duration=8))


        #rospy.sleep(st.send_jtp_list_synchronous(*am.boring_movement_variant()))

        #JTP.extend_base_list(jtp_flow_data, *am.movePoseSplit(am.pose_relaxed_arms_side,
        #                                                      am.pose_boring_walk_front, duration=3, times=4))

        '''
        #######################################################################
        # PEN TESTING !!!        DO NOT EXECUTE WITH NORMAL POSE SETTINGS !!! #
        # PEN TESTING !!!        DO NOT EXECUTE WITH NORMAL POSE SETTINGS !!! #
        # PEN TESTING !!!        DO NOT EXECUTE WITH NORMAL POSE SETTINGS !!! #
        #######################################################################


        JTP.extend_base_list(jtp_flow_data, *am.movePoseSplit(am.pose_relaxed_arms_side,
                                                              am.pose_marsh_walk_front, duration=1.75, times=3))

        JTP.extend_base_list(jtp_flow_data, *am.movePoseSplit(am.pose_relaxed_arms_side,
                                                              am.pose_marsh_walk_front, duration=1.5, times=3))

        JTP.extend_base_list(jtp_flow_data, *am.movePoseSplit(am.pose_relaxed_arms_side,
                                                              am.pose_marsh_walk_front, duration=1.25, times=3))

        JTP.extend_base_list(jtp_flow_data, *am.movePoseSplit(am.pose_relaxed_arms_side,
                                                              am.pose_marsh_walk_front, duration=1, times=3))

        JTP.extend_base_list(jtp_flow_data, *am.movePoseSplit(am.pose_relaxed_arms_side,
                                                              am.pose_marsh_walk_front, duration=2, times=3))


        #######################################################################
        # PEN TESTING !!!        DO NOT EXECUTE WITH NORMAL POSE SETTINGS !!! #
        # PEN TESTING !!!        DO NOT EXECUTE WITH NORMAL POSE SETTINGS !!! #
        # PEN TESTING !!!        DO NOT EXECUTE WITH NORMAL POSE SETTINGS !!! #
        #######################################################################


        JTP.extend_base_list(jtp_flow_data, *am.movePose(am.pose_home, 4))
        radians = m.radians(40)
        basetime = 3
        joint = 'p4'
        while basetime >= 0.5:
            print basetime
            JTP.extend_base_list(jtp_flow_data, *am.movePose({joint: radians*-1}, basetime))
            JTP.extend_base_list(jtp_flow_data, *am.movePose({joint: radians}, basetime*2))
            JTP.extend_base_list(jtp_flow_data, *am.movePose({joint: 0}, basetime))
            basetime /= 2.0

        '''



        #JTP.extend_base_list(jtp_flow_data, *am.movePose(am.pose_relaxed_arms_side, 3))
        #JTP.extend_base_list(jtp_flow_data, *am.movePose(am.pose_home, 6))

        rospy.sleep(st.send_jtp_list_synchronous(*jtp_flow_data))


        print
        print '*%s*' % ('-'*17)
        print '| SCRIPT FINISHED |'
        print '*%s*' % ('-'*17)
        print


