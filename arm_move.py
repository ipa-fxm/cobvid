#!/usr/bin/env python2
#try:
import roslib
try:
    isLive=True
    roslib.load_manifest('cobvid')
except:
    isLive=False

import rospy
import actionlib
import control_msgs.msg
import trajectory_msgs.msg

from actionlib import simple_action_client
from control_msgs.msg import JointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryAction

from trajectory_msgs.msg import JointTrajectoryPoint
import numpy as np


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
    def _updateTimes(jtp_list):
        cur_time = 0
        for jtp in jtp_list:
            cur_time += jtp.time_from_start
            jtp.point.time_from_start = rospy.Duration(cur_time)

    @staticmethod
    def get_point_list(jtp_list):
        JTP._updateTimes(jtp_list)
        return [jtp.point for jtp in jtp_list]

    def __repr__(self):
        return 'TimeFromstart: %s - Positions: %s - Velocities: %s' % \
               (self.point.time_from_start, self.point.positions, self.point.velocities)

class synchronous_traj():
    def __init__(self):

        #initializing the ROS node
        rospy.init_node('arm_traj_action')
        rospy.sleep(0.5)
        self.action_client_right = actionlib.SimpleActionClient('/arm_right/joint_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.action_client_left = actionlib.SimpleActionClient('/arm_left/joint_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.action_client_right.wait_for_server()
        self.action_client_left.wait_for_server()

    def send_jtp_list_left(self, jtp_list):
        # Creates the goal object to pass to the server
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint', 'arm_left_4_joint',
                                       'arm_left_5_joint', 'arm_left_6_joint', 'arm_left_7_joint']
        goal.trajectory.points = JTP.get_point_list(jtp_list)
        self.action_client_left.send_goal(goal)


    def send_jtp_list_right(self, jtp_list):
        # Creates the goal object to pass to the server
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ['arm_right_1_joint', 'arm_right_2_joint', 'arm_right_3_joint', 'arm_right_4_joint',
                                       'arm_right_5_joint', 'arm_right_6_joint', 'arm_right_7_joint']
        goal.trajectory.points = JTP.get_point_list(jtp_list)
        self.action_client_right.send_goal(goal)



if __name__=='__main__':
    import math as m


    front     = {'p1': m.radians(-50), 'p2': m.radians(80), 'p3': m.radians(25), 'p4': m.radians(50)}
    front_cp1 = {'p1': m.radians(-90), 'p2': m.radians(60), 'p3': m.radians(-45), 'p4': m.radians(40)}
    back_cp1 = {'p1': m.radians(-100), 'p2': m.radians(50), 'p3': m.radians(20), 'p4': m.radians(45)}
    back      = {'p1': m.radians(-110), 'p2': m.radians(50), 'p3': m.radians(20), 'p4': m.radians(60)}
    #JTP()
    jtp_list1 = list()
    jtp_list1.append(JTP(rel_time=8))
    jtp_list1.append(JTP(rel_time=4, **front))
    jtp_list1.append(JTP(rel_time=2, **front_cp1))
    jtp_list1.append(JTP(rel_time=2, **back_cp1))
    jtp_list1.append(JTP(rel_time=3, **back))
    jtp_list1.append(JTP(rel_time=2, **back_cp1))
    jtp_list1.append(JTP(rel_time=2, **front_cp1))
    jtp_list1.append(JTP(rel_time=3, **front))



    back_left =      {'p1': m.radians(35),  'p2': m.radians(-65), 'p3':  m.radians(89),  'p4': m.radians(60)}
    norm_left =      {'p1': m.radians(55),  'p2': m.radians(-58), 'p3':  m.radians(90),  'p4': m.radians(70)}
    norm_front_cp1 = {'p1': m.radians(85),  'p2': m.radians(-54), 'p3': m.radians(130),  'p4': m.radians(74)}
    front_left =     {'p1': m.radians(120), 'p2': m.radians(-65), 'p3': m.radians(160),  'p4': m.radians(75)}
    time_delta = 2

    jtp_list_left2 = list()
    #jtp_list_left2.append(JTP(rel_time=8))

    jtp_list_left2.append(JTP(rel_time=4, **norm_left))
    for i in range(8):
        jtp_list_left2.append(JTP(rel_time=time_delta, **back_left))
        jtp_list_left2.append(JTP(rel_time=time_delta, **norm_left))
        jtp_list_left2.append(JTP(rel_time=time_delta, **norm_front_cp1))
        jtp_list_left2.append(JTP(rel_time=time_delta, **front_left))
        jtp_list_left2.append(JTP(rel_time=time_delta, **norm_front_cp1))
        jtp_list_left2.append(JTP(rel_time=time_delta, **norm_left))

    time_delta = 4
    jtp_list_left3 = list()
    jtp_list_left3.append(JTP(rel_time=8))
    #for i in range(3):
    #    jtp_list_left3.append(JTP(rel_time=time_delta, p1=m.radians(55), p2=m.radians(-60), p3=m.radians(90), p4=m.radians(70)))
    #    jtp_list_left3.append(JTP(rel_time=time_delta, p1=m.radians(100), p2=m.radians(-60), p3=m.radians(130), p4=m.radians(90)))


    jtp_list_right3 = list()
    jtp_list_right3.append(JTP(rel_time=8))

    #for i in range(3):
    #    jtp_list_right3.append(JTP(rel_time=time_delta, p1=m.radians(-100), p2=m.radians(60), p3=m.radians(-130), p4=m.radians(-90)))
    #    jtp_list_right3.append(JTP(rel_time=time_delta, p1=m.radians(-55), p2=m.radians(60), p3=m.radians(-90), p4=m.radians(-70)))




    jtp_list_right2 = list()


    jtp_list_right2.append(JTP(rel_time=8))
    jtp_list_right2.append(JTP(rel_time=8, p1=m.radians(-55), p2=m.radians(65), p3=m.radians(-90), p4=m.radians(-75)))

    jtp_list_left = jtp_list_left3
    jtp_list_right = jtp_list_right3




    if isLive:
        sync_traj = synchronous_traj()
        r = rospy.Rate(10)
        sync_traj.send_jtp_list_left(jtp_list_left)
        rospy.sleep(0.5)
        sync_traj.send_jtp_list_right(jtp_list_right)
        #sync_traj.send_goal()





