#!/usr/bin/env python2

import roslib
roslib.load_manifest('cobvid')
import rospy
import actionlib
import control_msgs.msg
import trajectory_msgs.msg

from actionlib import simple_action_client
from control_msgs.msg import JointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryAction

from trajectory_msgs.msg import JointTrajectoryPoint



class synchronous_traj():
  def __init__(self):

    #initializing the ROS node
    rospy.init_node('arm_traj_action')
    rospy.sleep(0.5)
    self.action_client = actionlib.SimpleActionClient('/arm_left/joint_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    self.action_client_left = actionlib.SimpleActionClient('/arm_left/joint_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    self.action_client.wait_for_server()
    self.action_client_left.wait_for_server()
    
  def send_goal(self):
        
    # Creates the goal object to pass to the server
    goal1 = control_msgs.msg.FollowJointTrajectoryGoal()
    goal1.trajectory.joint_names = ['arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint', 'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint', 'arm_left_7_joint']

    time1 = 6
    time2 = time1 + 5
    time3 = time2 + 1
    vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    point1 = trajectory_msgs.msg.JointTrajectoryPoint()    
    point1.positions = [0, 0, 0, 0, 0, 0, 0]
    #point1.velocities = vel
    point1.time_from_start = rospy.Duration(time1)

    point2 = trajectory_msgs.msg.JointTrajectoryPoint()
    point2.positions = [0, 0, 0, 0.5, 0, 0, 0]
    #point2.velocities = vel#[0.0, 0.0, 0.0, 0, 0.0, 0.0, 0.0]
    point2.time_from_start = rospy.Duration(time2)

    point3 = trajectory_msgs.msg.JointTrajectoryPoint()
    point3.positions = [0, 0, 0, 1, 0, 0, 0]
    point3.velocities = vel
    point3.time_from_start = rospy.Duration(time3)

    goal1.trajectory.points = [point1, point2, point3]

    print "sending goal"
    self.action_client_left.send_goal(goal1)#_and_wait(goal1)
    #rospy.sleep(0.1)
    
if __name__=='__main__':
  sync_traj = synchronous_traj()
  r = rospy.Rate(10)
  #while not rospy.is_shutdown():
  sync_traj.send_goal()
    #r.sleep()




