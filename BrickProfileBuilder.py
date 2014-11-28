#!/usr/bin/env python2
#-*- coding: utf-8 -*-

# ROS IMPORTS
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

import rospy
import rosnode
import actionlib

# ROS MSGS
from geometry_msgs.msg import Twist
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from actionlib_msgs.msg import GoalStatus
from cob_srvs.srv import Trigger

# IMPORT FOR CALCULATIONS
import numpy as np

from scipy import integrate
from scipy.misc import comb

# PYTHON IMPORTS
import time
import sys
import os
import string
import operator as op

class PrettyOutput(object):

    @staticmethod
    def attation_msg(info_msg, question_msg=''):
        length = 80
        pl = list()
        cnt = 0
        while len(info_msg):
            if ' ' not in info_msg or len(info_msg) < length:
                pl.append(info_msg)

                if len(question_msg):
                    pl.append('')
                    pl.append(question_msg + ' [Y/n]')
                break

            split = info_msg[:length].rsplit(' ', 1)
            line = split[0].strip()
            info_msg = info_msg[len(line):].strip()
            pl.append(line)

        sign = ['    #      ', '   # #     ', '  # | #    ', ' #  .  #   ', '#########  ']

        print '-'*(12 + length)

        for idx in range(max(map(len, [sign, pl]))):
            if idx < len(sign):
                print sign[idx],
                if not idx < len(pl):
                    print

            if idx < len(pl):
                print pl[idx]

        print '-'*(12 + length)

        if not len(question_msg):
            return False

        return True if raw_input('') != 'n' else False


class Plotter(object):
    def __init__(self, timeline, plot_profile=False, plot_map=False):

        import matplotlib.pyplot as plt
        import scipy
        timeline.syncTimeline()
        self.timeline = timeline

        max_samples = len(timeline)
        self.tdata = np.linspace(0, self.timeline.profile.sample_time * max_samples, max_samples)

        # distance / absphi
        TLXD = np.array([0], np.float)
        TLYD = np.array([0], np.float)
        TLTHD = np.array([0], np.float)

        self.TLXD = np.append(TLXD, integrate.cumtrapz(self.timeline.TLX, self.tdata))
        self.TLYD = np.append(TLYD, integrate.cumtrapz(self.timeline.TLY, self.tdata))
        self.TLTHD = np.append(TLTHD, integrate.cumtrapz(self.timeline.TLTH, self.tdata))

        # acceleration
        TLXA = np.array([0], np.float)
        TLYA = np.array([0], np.float)
        TLTHA = np.array([0], np.float)

        self.TLXA = np.append(TLXA, scipy.diff(self.timeline.TLX))
        self.TLYA = np.append(TLYA, scipy.diff(self.timeline.TLY))
        self.TLTHA = np.append(TLTHA, scipy.diff(self.timeline.TLTH))

        if plot_profile:
            self.plot_profile()

        if plot_map:
            self.plot_map()

        if plot_map or plot_profile:
            plt.show()


    def plot_profile(self):

        import matplotlib.pyplot as plt
        import matplotlib.collections as collections

        fig, (ax1, ax2, ax3) = plt.subplots(nrows=3, ncols=1)
        legend = ['x', 'y', '$\\theta$']

        ax1.plot(self.tdata, self.TLXD)
        ax1.plot(self.tdata, self.TLYD)
        ax1.plot(self.tdata, self.TLTHD)
        ax1.legend(legend, loc=2)
        ax1.set_title('Distance')
        ax1.grid(True)

        ##################################################

        accwarn1 = np.abs(self.timeline.TLX) > self.timeline.profile.max_linear_velocity
        accwarn2 = np.abs(self.timeline.TLY) > self.timeline.profile.max_linear_velocity
        accwarn3 = np.abs(self.timeline.TLTH) > self.timeline.profile.max_angular_velocity
        accwarn = np.logical_or(accwarn1, accwarn2)
        accwarn = np.logical_or(accwarn, accwarn3)

        collection = collections.BrokenBarHCollection.span_where(self.tdata, ymin=-self.timeline.profile.max_linear_velocity, ymax=self.timeline.profile.max_linear_velocity, where=accwarn1, facecolor='blue', alpha=0.2)
        ax2.add_collection(collection)
        collection = collections.BrokenBarHCollection.span_where(self.tdata, ymin=-self.timeline.profile.max_linear_velocity, ymax=self.timeline.profile.max_linear_velocity, where=accwarn2, facecolor='green', alpha=0.2)
        ax2.add_collection(collection)
        collection = collections.BrokenBarHCollection.span_where(self.tdata, ymin=-self.timeline.profile.max_angular_velocity, ymax=self.timeline.profile.max_angular_velocity, where=accwarn3, facecolor='red', alpha=0.2)
        ax2.add_collection(collection)

        ax2.plot(self.tdata, self.timeline.TLX)
        ax2.plot(self.tdata, self.timeline.TLY)
        ax2.plot(self.tdata, self.timeline.TLTH)
        ax2.legend(legend, loc=2)
        ax2.set_title('Velocity')
        ax2.grid(True)

        ##################################################

        ax3.plot(self.tdata, self.TLXA)
        ax3.plot(self.tdata, self.TLYA)
        ax3.plot(self.tdata, self.TLTHA)
        ax3.legend(legend, loc=2)
        ax3.set_title('Acceleration')
        ax3.grid(True)

        accwarn1 = np.abs(self.TLXA) > self.timeline.profile.max_linear_acceleration
        accwarn2 = np.abs(self.TLYA) > self.timeline.profile.max_linear_acceleration
        accwarn3 = np.abs(self.TLTHA) > self.timeline.profile.max_angular_acceleration
        accwarn = np.logical_or(accwarn1, accwarn2)
        accwarn = np.logical_or(accwarn, accwarn3)

        collection = collections.BrokenBarHCollection.span_where(self.tdata, ymin=-self.timeline.profile.max_linear_acceleration, ymax=self.timeline.profile.max_linear_acceleration, where=accwarn1, facecolor='blue', alpha=0.2)
        ax3.add_collection(collection)
        collection = collections.BrokenBarHCollection.span_where(self.tdata, ymin=-self.timeline.profile.max_linear_acceleration, ymax=self.timeline.profile.max_linear_acceleration, where=accwarn2, facecolor='green', alpha=0.2)
        ax3.add_collection(collection)
        collection = collections.BrokenBarHCollection.span_where(self.tdata, ymin=-self.timeline.profile.max_angular_acceleration, ymax=self.timeline.profile.max_angular_acceleration, where=accwarn3, facecolor='red', alpha=0.2)
        ax3.add_collection(collection)

        plt.tight_layout()

    def plot_map(self):

        import matplotlib.pyplot as plt
        import matplotlib.collections as collections

        fig, (ax1) = plt.subplots(nrows=1, ncols=1)

        x = self.timeline.TLX * np.cos(self.TLTHD) - self.timeline.TLY * np.sin(self.TLTHD)
        y = self.timeline.TLY * np.cos(self.TLTHD) + self.timeline.TLX * np.sin(self.TLTHD)

        x = integrate.cumtrapz(x, self.tdata)
        y = integrate.cumtrapz(y, self.tdata)


        last_endpoint = 0
        color_scale = 0.5
        idx=0
        for idx, sec in enumerate(self.timeline.SECTIONS):
            name = sec['name']
            sp = sec['at_sample']

            ax1.text(x[sp], y[sp], '\n      %s'%name, fontsize=12, alpha=0.25)

            color = (color_scale, color_scale, 0) if idx%2 else (color_scale, 0, color_scale)
            ax1.plot(x[last_endpoint:sp], y[last_endpoint:sp], '-', color=color, alpha=0.4)

            last_endpoint = sp
        else:
            idx += 1
            color = (color_scale, color_scale, 0) if idx%2 else (color_scale, 0, color_scale)
            ax1.plot(x[last_endpoint:], y[last_endpoint:], '-', color=color, alpha=0.4)


        ax1.set_xlabel('x [m]')
        ax1.set_ylabel('y [m]')
        ax1.legend(['Path Map'])
        ax1.grid(True)

        accx = self.TLXA > 0
        decx = self.TLXA < 0

        accy = self.TLYA > 0
        decy = self.TLYA < 0

        accwarn1 = np.abs(self.TLXA) > self.timeline.profile.max_linear_acceleration
        accwarn2 = np.abs(self.TLYA) > self.timeline.profile.max_linear_acceleration
        accwarn3 = np.abs(self.TLTHA) > self.timeline.profile.max_angular_acceleration
        accwarn = np.logical_or(accwarn1, accwarn2)
        accwarn = np.logical_or(accwarn, accwarn3)

        marker_style = dict(alpha=0.4, markersize=10)


        for i in range(len(x)):
            if accwarn[i]:
                ax1.plot(x[i], y[i], '.', color=(0.8, 0, 0), alpha=0.1, markersize=50)

        for i in range(len(x))[::15]:
            if accx[i]:
                ax1.plot(x[i], y[i], '>', color=(0, 0.8, 0), **marker_style)
            if decx[i]:
                ax1.plot(x[i], y[i], '<', color=(0.8, 0, 0), **marker_style)

            if accy[i]:
                ax1.plot(x[i], y[i], '^', color=(0, 0.8, 0), **marker_style)
            if decy[i]:
                ax1.plot(x[i], y[i], 'v', color=(0.8, 0, 0), **marker_style)

        for i in range(len(x))[::int(self.timeline.profile.rate/2)]:
            text = '\n%s' % np.round(self.tdata[i], 1)
            ax1.text(x[i], y[i], text, fontsize=8, alpha=0.15)
            ax1.plot(x[i], y[i], '.', color=(0, 0, 0), alpha=0.15)


        plt.tight_layout()
        plt.axis('equal')



class ROSBridge(object):
    ACTIVE_TASK = False

    class Dummy(object):
        def __init__(self, topic_name):
            self.topic_name = topic_name

        def publish(self, msg):
            print 'FAKE: %s:   %s' % (self.topic_name, msg)

        def send_jtp_list(self, jtp_list, is_left_arm=True):
            print 'FAKE: %s_%s:   %s' % (self.topic_name, 'LEFT' if is_left_arm else 'RIGHT', jtp_list)
            return 1


    def __init__(self, fakerun=False, exec_base=False, exec_arm_left=False, exec_arm_right=False):
        self.exec_base = exec_base
        self.exec_arm_left = exec_arm_left
        self.exec_arm_right = exec_arm_right

        BASE_CONTROLLER_TOPIC = '/base_controller/command_direct'
        ARM_LEFT_VELOCITY_TOPIC = '/arm_left/joint_group_velocity_controller/command'
        ARM_RIGHT_VELOCITY_TOPIC = '/arm_right/joint_group_velocity_controller/command'

        queue_size = 0

        self.base_publisher = rospy.Publisher(BASE_CONTROLLER_TOPIC, Twist, queue_size=queue_size) \
            if not fakerun and isLive and exec_base else ROSBridge.Dummy('BASE')

        self.arm_left_velocity_publisher = rospy.Publisher(ARM_LEFT_VELOCITY_TOPIC, Float64MultiArray, queue_size=queue_size) \
            if not fakerun and isLive and exec_arm_left else ROSBridge.Dummy('ARM_LEFT')

        self.arm_right_velocity_publisher = rospy.Publisher(ARM_RIGHT_VELOCITY_TOPIC, Float64MultiArray, queue_size=queue_size) \
            if not fakerun and isLive and exec_arm_right else ROSBridge.Dummy('ARM_RIGHT')

        if not fakerun and isLive:
            self.synchronous_trajectory = SynchronousTrajectory(init_arm_left=self.exec_arm_left, init_arm_right=self.exec_arm_right)
        else:
            self.synchronous_trajectory = ROSBridge.Dummy('ARM_GOAL')


    def _exec_arm(self, timeline, arm_goal_timeline, arm_velocity_timeline, is_left_arm, step, synchronous_trajectory, publisher):
        if arm_goal_timeline[step] is not None:
            synchronous_trajectory.send_jtp_list(arm_goal_timeline[step], is_left_arm=is_left_arm)

        if arm_velocity_timeline[step][7]:
            msg = Float64MultiArray(data=arm_velocity_timeline[step][0:7])
            publisher.publish(msg)

    def block_arm_velocity_timeline_for_goals(self, timeline, arm_goal_timeline, arm_velocity_timeline):
        for idx, jtp_list in enumerate(arm_goal_timeline):
            if jtp_list:
                goal_duration = JTP.get_total_time_duration(jtp_list)
                timeoutSamples = int(timeline.calc_samples(timeline.profile.switch_vel_to_goal_timeout))
                blockedSamples = int(timeline.calc_samples(goal_duration))
                arm_velocity_timeline[idx-timeoutSamples:idx+blockedSamples, 7] = 0

                data_loss = arm_velocity_timeline[idx-timeoutSamples:idx+blockedSamples] != 0
                is_data_loss = bool(data_loss.sum())

                info_msg = 'possible loss of data due to blocking arm velocity\'s for goal at sample %d' % (idx+1)
                question_msq = 'EXIT TASK?'
                if is_data_loss and PrettyOutput.attation_msg(info_msg, question_msq):
                    self._exit_task()

    def _exit_task(self):
        PrettyOutput.attation_msg('EXITING TASK')
        ROSBridge.ACTIVE_TASK = False
        exit()

    def exec_timeline(self, timeline):
        if ROSBridge.ACTIVE_TASK:
            PrettyOutput.attation_msg('ANOTHER TASK STILL RUNNING - SKIP EXECUTION OF NEW TASK')
            return

        ROSBridge.ACTIVE_TASK = True

        timeline.syncTimeline()

        self.block_arm_velocity_timeline_for_goals(timeline=timeline, arm_goal_timeline=timeline.ARML_GOAL,
                                                   arm_velocity_timeline=timeline.ARML_VEL)

        self.block_arm_velocity_timeline_for_goals(timeline=timeline, arm_goal_timeline=timeline.ARMR_GOAL,
                                                   arm_velocity_timeline=timeline.ARMR_VEL)

        for step in range(timeline.get_max_length_from_timelines()):
            if self.exec_base:
                twist = Twist()
                twist.linear.x = timeline.TLX[step]
                twist.linear.y = timeline.TLY[step]
                twist.angular.z = timeline.TLTH[step]
                self.base_publisher.publish(twist)

            if self.exec_arm_left or self.exec_arm_right:
                if self.exec_arm_left:
                    self._exec_arm(timeline=timeline, arm_goal_timeline=timeline.ARML_GOAL,
                                   arm_velocity_timeline=timeline.ARML_VEL, is_left_arm=True, step=step,
                                   synchronous_trajectory=self.synchronous_trajectory,
                                   publisher=self.arm_left_velocity_publisher)

                if self.exec_arm_right:
                    self._exec_arm(timeline=timeline, arm_goal_timeline=timeline.ARMR_GOAL,
                                   arm_velocity_timeline=timeline.ARMR_VEL, is_left_arm=False, step=step,
                                   synchronous_trajectory=self.synchronous_trajectory,
                                   publisher=self.arm_right_velocity_publisher)

            rospy.sleep(timeline.profile.sample_time)

        ROSBridge.ACTIVE_TASK = False

class ServiceHandler(object):
    def __init__(self):
        self.is_fakerun = False

        self.is_service_mode = False

        self.is_plot = False
        self.plot_map = False
        self.plot_profile = False

        self.is_ros = False
        self.is_ros_arm_left = False
        self.is_ros_arm_right = False
        self.is_ros_base = False

        self.init_startup_args()
        self.print_startup_args()
        self.init_management_services()

    def init_management_services(self):
        if not self.is_service_mode:
            return

        rospy.init_node('scenario')
        rospy.Service('/scenario/restart', Trigger, self._inplace_restart)
        rospy.Service('/scenario/status', Trigger, self.print_startup_args)

        rospy.Service('/scenario/enable_fakerun', Trigger, self._enable_fakerun)
        rospy.Service('/scenario/disable_fakerun', Trigger, self._disable_fakerun)

        rospy.Service('/scenario/enable_base', Trigger, self._enable_base)
        rospy.Service('/scenario/disable_base', Trigger, self._disable_base)

        rospy.Service('/scenario/enable_arm_left', Trigger, self._enable_arm_left)
        rospy.Service('/scenario/disable_arm_left', Trigger, self._disable_arm_left)

        rospy.Service('/scenario/enable_arm_right', Trigger, self._enable_arm_right)
        rospy.Service('/scenario/disable_arm_right', Trigger, self._disable_arm_right)

    def init_startup_args(self):
        self.is_fakerun = '-fakerun' in sys.argv
        self.is_service_mode = '-servicemode' in sys.argv

        self.is_plot = '-plot' in sys.argv
        if self.is_plot:
            idx = sys.argv.index('-plot') + 1
            self.plot_map = 'map' in sys.argv[idx:idx+2]
            self.plot_profile = 'profile' in sys.argv[idx:idx+2]

        self.is_ros = '-ros' in sys.argv
        if self.is_ros:
            idx = sys.argv.index('-ros') + 1
            self.is_ros_arm_left = 'arm_left' in sys.argv[idx:idx+3]
            self.is_ros_arm_right = 'arm_right' in sys.argv[idx:idx+3]
            self.is_ros_base = 'base' in sys.argv[idx:idx+3]

    def print_startup_args(self, *_):
        print
        print
        print 'FAKERUN:      ', self.is_fakerun
        print
        print 'SERVICE MODE: ', self.is_service_mode
        print
        print 'ROS ENABLED:  ', self.is_ros
        print '    ARM_LEFT: ', self.is_ros_arm_left
        print '    ARM RIGHT:', self.is_ros_arm_right
        print '    BASE:     ', self.is_ros_base
        print

    def callback_creator(self, service_name, func_list, bound_timeline_object):
        def callback_function(_):
            PrettyOutput.attation_msg('CALLBACK TRIGGERED: %s' % service_name)
            bound_timeline_object.clear_data()
            [fnc() for fnc in func_list]
            self.execute_timeline(bound_timeline_object, is_callback=True)
            PrettyOutput.attation_msg('END OF CALLBACK: %s' % service_name)
        return callback_function

    def add_service_callback(self, service_name, func_list, bound_timline_object):
        if not isinstance(func_list, list):
            func_list = [func_list]
        rospy.Service(service_name, Trigger, self.callback_creator(service_name, func_list, bound_timline_object))

    def do_listen(self):
        if not self.is_service_mode:
            PrettyOutput.attation_msg('LISTENING ONLY IN SERVICEMODE')
            return
        rospy.spin()

    def _inplace_restart(self, *args):
        for _ in range(50): print
        PrettyOutput.attation_msg('RESTARTING APPLICATION')
        os.execv(__file__, sys.argv)

    def _enable_argv(self, argname, argvar):
        if self.is_ros and argvar:
            return

        if self.is_ros:
            idx = sys.argv.index('-ros')
            sys.argv.insert(idx+1, argname)
        else:
            sys.argv.append('-ros')
            sys.argv.append(argname)

        self._inplace_restart()

    def _disable_argv(self, argname, argvar):
        if not argvar:
            return

        idx = sys.argv.index(argname)
        sys.argv.pop(idx)

        self.init_startup_args()

        if not (self.is_ros_base or self.is_ros_arm_left or self.is_ros_arm_right):
            idx = sys.argv.index('-ros')
            sys.argv.pop(idx)

        self._inplace_restart()

    def _enable_base(self, *args):
        self._enable_argv('base', self.is_ros_base)

    def _disable_base(self, *args):
        self._disable_argv('base', self.is_ros_base)

    def _enable_fakerun(self, *args):
        if self.is_fakerun:
            return

        sys.argv.append('-fakerun')

        self._inplace_restart()

    def _disable_fakerun(self, *args):
        if not self.is_fakerun:
            return

        idx = sys.argv.index('-fakerun')
        sys.argv.pop(idx)

        self._inplace_restart()

    def _enable_arm_left(self, *args):
        self._enable_argv('arm_left', self.is_ros_arm_left)

    def _disable_arm_left(self, *args):
        self._disable_argv('arm_left', self.is_ros_arm_left)

    def _enable_arm_right(self, *args):
        self._enable_argv('arm_right', self.is_ros_arm_right)

    def _disable_arm_right(self, *args):
        self._disable_argv('arm_right', self.is_ros_arm_right)

    def execute_timeline(self, timeline, is_callback=False):
        if not is_callback and self.is_service_mode:
            PrettyOutput.attation_msg('SERVICEMODE ENABLED - DIRECT EXECUTION FORBIDDEN')
            return

        if self.is_plot:
            Plotter(timeline, plot_profile=self.plot_profile, plot_map=self.plot_map)

        if self.is_ros:
            bridge = ROSBridge(fakerun=self.is_fakerun,
                               exec_base=self.is_ros_base,
                               exec_arm_left=self.is_ros_arm_left,
                               exec_arm_right=self.is_ros_arm_right)

            bridge.exec_timeline(timeline)


class Profile(object):
    def __init__(self, rate, max_linear_velocity, max_angular_velocity, max_linear_acceleration, max_angular_acceleration, switch_vel_to_goal_timeout):
        self.rate = float(rate)  # [Hz]
        self.sample_time = 1.0 / rate  # [s]

        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        self.max_linear_acceleration = max_linear_acceleration
        self.max_angular_acceleration = max_angular_acceleration
        self.switch_vel_to_goal_timeout = switch_vel_to_goal_timeout


class Timeline(object):
    def __init__(self, profile):
        self.profile = profile
        self.TLX = None
        self.TLY = None
        self.TLTH = None
        self.ARML_GOAL = None
        self.ARMR_GOAL = None
        self.ARML_VEL = None
        self.ARMR_VEL = None
        self.SECTIONS = None

        self.clear_data()

    def clear_data(self):
        self.TLX = np.array([], np.float64)
        self.TLY = np.array([], np.float64)
        self.TLTH = np.array([], np.float64)
        self.ARML_GOAL = list()
        self.ARMR_GOAL = list()
        self.ARML_VEL = np.ndarray((0, 8), np.float64)
        self.ARMR_VEL = np.ndarray((0, 8), np.float64)
        self.SECTIONS = list()


    def appendX(self, data):
        self.TLX = np.append(self.TLX, data)

    def appendY(self, data):
        self.TLY = np.append(self.TLY, data)

    def appendTH(self, data):
        self.TLTH = np.append(self.TLTH, data)

    def appendArms(self, arm_data, fillToMax=False):
        arm_left_data, arm_right_data = arm_data
        self.appendArmLeft(arm_left_data=arm_left_data)
        self.appendArmRight(arm_right_data=arm_right_data)
        self._fillArmGoalTimelinesForMinDuration(arm_data, max if fillToMax else min)

    def _fillArmGoalTimelinesForMinDuration(self, arm_data, mode, fillLeft=True, fillRight=True):
        min_duration = mode(map(JTP.get_total_time_duration, arm_data))
        min_samples = min_duration / self.profile.sample_time
        fill_data = [None]*int(min_samples-1)

        if fillLeft:
            self.ARML_GOAL.extend(fill_data)
        if fillRight:
            self.ARMR_GOAL.extend(fill_data)

        self.syncTimelineArmGoal()

    def appendArmLeft(self, arm_left_data):
        if not arm_left_data:
            return
        self.ARML_GOAL.append(arm_left_data)

    def appendArmRight(self, arm_right_data):
        if not arm_right_data:
            return
        self.ARMR_GOAL.append(arm_right_data)

    def appendVelArm(self, j1=None,  j2=None,  j3=None,  j4=None,  j5=None,  j6=None,  j7=None):
        self.appendVelArmLeft(j1=j1, j2=j2, j3=j3, j4=j4, j5=j5, j6=j6, j7=j7)
        self.appendVelArmRight(j1=j1, j2=j2, j3=j3, j4=j4, j5=j5, j6=j6, j7=j7)

    def appendVelArmLeft(self, j1=None,  j2=None,  j3=None,  j4=None,  j5=None,  j6=None,  j7=None):
        data = self._createVelArmData(j1=j1, j2=j2, j3=j3, j4=j4, j5=j5, j6=j6, j7=j7)
        self.ARML_VEL = np.append(self.ARML_VEL, data, axis=0)

    def appendVelArmRight(self, j1=None,  j2=None,  j3=None,  j4=None,  j5=None,  j6=None,  j7=None):
        data = self._createVelArmData(j1=j1, j2=j2, j3=j3, j4=j4, j5=j5, j6=j6, j7=j7)
        self.ARMR_VEL = np.append(self.ARMR_VEL, data, axis=0)

    def appendSwitchVelToGoalTimeout(self):
        data = self.lin(0, self.profile.switch_vel_to_goal_timeout)
        self.appendVelArmLeft(j1=data)
        self.appendVelArmRight(j1=data)

    def _createVelArmData(self, j1=None,  j2=None,  j3=None,  j4=None,  j5=None,  j6=None,  j7=None):
        joints = [j1, j2, j3, j4, j5, j6, j7]
        filled_joints = [jn for jn in joints if jn is not None]
        maxSamples = max(map(len, filled_joints)) if filled_joints else 0

        if not maxSamples:
            return

        data = np.zeros((maxSamples, 8), np.float64)
        for n, j in enumerate(joints):
            if j is None:
                continue

            data[..., n] = j
            data[..., 7] = 1
        return data

    def syncTimeline(self):
        self.syncTimelineBase()
        self.syncTimelineArmVelocity()
        self.syncTimelineArmGoal()

    def syncTimelineBase(self):
        self.TLX, self.TLY, self.TLTH = self._evenMaxSamples(np.zeros, [np.float64], 0, self.TLX, self.TLY, self.TLTH)

    def syncTimelineArmVelocity(self):
        self.ARML_VEL, self.ARMR_VEL = self._evenMaxSamples(np.zeros, [np.float64], 8, self.ARML_VEL, self.ARMR_VEL)

    def syncTimelineArmGoal(self):
        self.ARML_GOAL, self.ARMR_GOAL = self._evenMaxSamples(self._generatePythonList, [None], 0, self.ARML_GOAL, self.ARMR_GOAL)

    def _generatePythonList(self, max_samples, *args):
        return list(args)*max_samples

    def _evenMaxSamples(self, list_generator, list_generator_args, data_array_size, *args):
        args = list(args)
        max_samples = self.get_max_length_from_timelines()
        for idx, sample_line in enumerate(args):
            remaining_samples = max_samples - len(sample_line)
            shape = (remaining_samples, data_array_size) if data_array_size > 0 else (remaining_samples)
            fill_data = list_generator(shape, *list_generator_args)

            if isinstance(args[idx], np.ndarray):
                args[idx] = np.append(sample_line, fill_data, axis=0)
            elif isinstance(args[idx], list):
                args[idx].extend(fill_data)

        return args

    def get_max_length_from_timelines(self):
        timelines = [self.TLX, self.TLY, self.TLTH, self.ARML_GOAL, self.ARMR_GOAL, self.ARML_VEL, self.ARMR_VEL]
        return max(map(len, timelines))

    def __len__(self):
        return self.get_max_length_from_timelines()

    def new_section(self, name='', sync_timelines=True):
        if sync_timelines:
            self.syncTimeline()
        self.SECTIONS.append({'name': name, 'at_sample': len(self)})

    def appendReversePath(self):
        self.syncTimeline()
        self.appendX(self.TLX[::-1]*-1)
        self.appendY(self.TLY[::-1]*-1)
        self.appendTH(self.TLTH[::-1]*-1)


class Bezier(object):
    def __init__(self, profile):
        self.profile = profile

    def bernstein_poly(self, i, n, t):
        return comb(n, i) * ( t**(n-i) ) * (1 - t)**i

    def bezier_curve(self, points, nTimes=1000):
        nPoints = len(points)
        xPoints, yPoints = map(np.array, zip(*points))
        t = np.linspace(0.0, 1.0, nTimes)

        polynomial_array = np.array([self.bernstein_poly(i, nPoints-1, t) for i in range(0, nPoints)])

        xvals = np.dot(xPoints, polynomial_array)
        yvals = np.dot(yPoints, polynomial_array)

        return xvals[::-1], yvals[::-1]

    def createBezier(self, points, duration=0):
        if isinstance(points, list):
            points = np.array(points, np.float)

        nSamples=duration*self.profile.rate


        xvals, yvals = self.bezier_curve(points, nTimes=nSamples)

        xdiff = np.diff(xvals)
        ydiff = np.diff(yvals)

        rel_distance = np.sqrt(xdiff**2 + ydiff**2)
        abs_phi = np.arctan2(xdiff, ydiff)



        rel_phi = np.diff(abs_phi)
        corridx = [(idx+1, 1 if phi < 0 else -1) for idx, phi in enumerate(rel_phi) if abs(phi) > 6]
        for k, v in corridx:
            abs_phi[k:] += np.pi*2*v
        rel_phi = np.diff(abs_phi)

        velocity = rel_distance / self.profile.sample_time
        theta_velocity = rel_phi / self.profile.sample_time


        return velocity, theta_velocity*-1



class JTP(object):

    def __init__(self, rel_time=0,
                 p1=None, p2=None, p3=None, p4=None, p5=None, p6=None, p7=None,
                 v1=None, v2=None, v3=None, v4=None, v5=None, v6=None, v7=None, jtp=None):

        positions = [p1, p2, p3, p4, p5, p6, p7]
        velocities = [v1, v2, v3, v4, v5, v6, v7]
        isPosSet = bool(sum([int(p is not None) for p in positions]))
        isVelSet = bool(sum([int(p is not None) for p in velocities]))

        for k, v in enumerate(positions):
            positions[k] = v if v else jtp.point.positions[k] if jtp else 0

        self.point = JointTrajectoryPoint()

        if isinstance(jtp, JTP):
            self.point.positions = jtp.point.positions
        self.point.positions = positions

        if isVelSet:
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
        velocities = [v*-1 for v in jtp.point.velocities]

        args = list()
        args.extend(positions)
        args.extend(velocities)

        return JTP(jtp.time_from_start, *args)


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
               (self.point.time_from_start, [np.round(np.degrees(p)) for p in self.point.positions], self.point.velocities)
               #(self.point.time_from_start, self.point.positions, self.point.velocities)


class SynchronousTrajectory():
    def __init__(self, ros_rate=10, init_arm_left=False, init_arm_right=False, timeout=5):
        self.init_arm_left = init_arm_left
        self.init_arm_right = init_arm_right

        ARM_LEFT_FOLLOW_JOINT_TRAJECTORY_TOPIC = '/arm_left/joint_trajectory_controller/follow_joint_trajectory'
        ARM_RIGHT_FOLLOW_JOINT_TRAJECTORY_TOPIC = '/arm_right/joint_trajectory_controller/follow_joint_trajectory'

        self.goal_status_dict = self._resolve_goal_stats_codes()

        if init_arm_left:
            self.action_client_left = actionlib.SimpleActionClient(ARM_LEFT_FOLLOW_JOINT_TRAJECTORY_TOPIC, FollowJointTrajectoryAction)
            if not self.action_client_left.wait_for_server(timeout=rospy.Duration(timeout)):
                if PrettyOutput.attation_msg('CAN NOT INITIALIZE LEFT ARM', 'EXIT APPLICATION?'):
                    print 'exiting...'
                    exit()

        if init_arm_right:
            self.action_client_right = actionlib.SimpleActionClient(ARM_RIGHT_FOLLOW_JOINT_TRAJECTORY_TOPIC, FollowJointTrajectoryAction)
            if not self.action_client_right.wait_for_server(timeout=rospy.Duration(timeout)):
                if PrettyOutput.attation_msg('CAN NOT INITIALIZE RIGHT ARM', 'EXIT APPLICATION?'):
                    print 'exiting...'
                    exit()

    def _resolve_goal_stats_codes(self):
        CODENAMES = [code for code in dir(GoalStatus) if not code.startswith('_') and code[0] in string.ascii_uppercase]
        CODENUMS = map(getattr, [GoalStatus] * len(CODENAMES), CODENAMES)
        goal_status_dict = dict()
        map(lambda num, name: goal_status_dict.update({num:name}) ,CODENUMS, CODENAMES)
        return goal_status_dict


    def get_joint_names(self, is_left_arm=True):
        joint_name_pattern = ['arm_%s_%d_joint']*7
        direction = 'left' if is_left_arm else 'right'
        return [pattern % (direction, idx+1) for idx, pattern in enumerate(joint_name_pattern)]

    def send_jtp_list_synchronous(self, jtp_list_left, jtp_list_right):
        self.send_jtp_list(jtp_list_left, is_left_arm=True)
        self.send_jtp_list(jtp_list_right, is_left_arm=False)
        return JTP.get_total_time_duration(jtp_list_left, jtp_list_right)

    def send_jtp_list(self, jtp_list, is_left_arm=True):
        if is_left_arm and not self.init_arm_left:
            return

        if not is_left_arm and not self.init_arm_right:
            return

        print 'Sending following JTP-Points to %s arm:' % ('left' if is_left_arm else 'right')
        for jtp in jtp_list:
            print jtp

        action_client = self.action_client_left if is_left_arm else self.action_client_right
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.get_joint_names(is_left_arm)
        goal.trajectory.points = JTP.get_point_list(jtp_list)
        action_client.send_goal(goal=goal)
        #rospy.sleep(0.05)
        return JTP.get_total_time_duration(jtp_list)

class ArmMovement(object):
    pose_home = {'p1': 0, 'p2': 0, 'p3': 0, 'p4': 0, 'p5': 0, 'p6': 0, 'p7': 0,
                 'v1': 0, 'v2': 0, 'v3': 0, 'v4': 0, 'v5': 0, 'v6': 0, 'v7': 0}

    pose_boring_walk_back = {'p1': np.radians(-55), 'p2': np.radians(-95),
                             'p3': np.radians(60), 'p4': np.radians(95),
                             'p5': np.radians(60), 'p6': np.radians(40),
                             'v1': 0, 'v2': 0, 'v3': 0, 'v4': 0, 'v5': 0, 'v6': 0, 'v7': 0}


    pose_boring_walk_front_back_c1 = {'p1': np.radians(-80), 'p2': np.radians(-74),
                                      'p3': np.radians(100), 'p4': np.radians(94),
                                      'p5': np.radians(60), 'p6': np.radians(40)}


    pose_boring_walk_front = {'p1': np.radians(-110), 'p2': np.radians(-75),
                              'p3': np.radians(147), 'p4': np.radians(90),
                              'p5': np.radians(60), 'p6': np.radians(40),
                              'v1': 0, 'v2': 0, 'v3': 0, 'v4': 0, 'v5': 0, 'v6': 0, 'v7': 0}


    pose_run_arms = {'p1': np.radians(-60), 'p2': np.radians(-75),
                     'p3': np.radians(80), 'p4': np.radians(105),
                     'p5': np.radians(40), 'p6': np.radians(20)}


    pose_cheer_arms_up = {'p1': np.radians(-172), 'p2': np.radians(-83),
                          'p3': np.radians(115), 'p4': np.radians(100),
                          'p5': np.radians(0), 'p6': np.radians(20)}

    bridge_pose_boring_walk_c1_to_waiting_arms_side = {'p1': np.radians(-75), 'p2': np.radians(-72),
                                                       'p3': np.radians(103), 'p4': np.radians(92),
                                                       'p5': np.radians(35), 'p6': np.radians(45)}


    pose_waiting_arms_side = {'p1': np.radians(-45), 'p2': np.radians(-60),
                              'p3': np.radians(105), 'p4': np.radians(90),
                              'p5': np.radians(10), 'p6': np.radians(50)}

    # ROSE GREIFEN

    pose_right_folded_back = {'p1': np.radians(55), 'p2': np.radians(90),
                              'p3': np.radians(0), 'p4': np.radians(80),
                              'p5': np.radians(55), 'p6': np.radians(40)}


    pose_folded_grip_right_c1 = {'p1': np.radians(57), 'p2': np.radians(96),
                                 'p3': np.radians(-17), 'p4': np.radians(0),
                                 'p5': np.radians(14), 'p6': np.radians(-90)}

    pose_folded_grip_right_c2 = {'p1': 1.59, 'p2': 1.12,
                                 'p3': -1.70, 'p4': -2,
                                 'p5': 0, 'p6': -1.08}

    pose_folded_grip_right_c3 = {'p1': 2.34, 'p2': 1.20,
                                 'p3': -2.45, 'p4': -1.8,
                                 'p5': 0.38, 'p6': -0.5}

    pose_folded_grip_right_c4 = {'p1': 3.14, 'p2': 1.66,
                                 'p3': -1.93, 'p4': -1.37,
                                 'p5': 0.31, 'p6': 1.0}

    pose_grip_rose_right = {'p1': 3.38, 'p2': 1.83,
                                 'p3': -1.93, 'p4': -0.45,
                                 'p5': 0.31, 'p6': 0.55}

    pose_folded_grip_right_c6 = {'p1': 3.57, 'p2': 1.83,
                                 'p3': -1.93, 'p4': -0.20,
                                 'p5': 0.31, 'p6': 0.13}

    pose_carry_rose_front_right = {'p1': 2.34, 'p2': 1.20,
                             'p3': -2.45, 'p4': -1.96,
                             'p5': 0.38, 'p6': 0.59}


    pose_cheer_drum = {'p1': -2.37, 'p2': -1.39,
                       'p3': 1.97, 'p4': 1.37,
                       'p5': 0, 'p6': 0.18}

    pose_cheer_turn = {'p1': -4.13, 'p2': -1.20,
                       'p3': 2.64, 'p4': 1.17,
                       'p5': 0, 'p6': 0.35}


    def __init__(self, profile):
        self.profile = profile

        '''
        self.pose_home = {'p1': 0, 'p2': 0, 'p3': 0, 'p4': 0, 'p5': 0, 'p6': 0, 'p7': 0,
                          'v1': 0, 'v2': 0, 'v3': 0, 'v4': 0, 'v5': 0, 'v6': 0, 'v7': 0}

        #self.pose_cross_arms_behind = {'p1': np.radians(-40), 'p2': np.radians(-65),
        #                               'p3': np.radians(170), 'p4': np.radians(70)}

        #self.pose_relaxed_arms_side = {'p1': np.radians(-55), 'p2': np.radians(-60),
        #                               'p3': np.radians(90), 'p4': np.radians(70)}


        self.pose_right_folded_back = {'p1': np.radians(-45), 'p2': np.radians(90),
                                       'p3': np.radians(0), 'p4': np.radians(80),
                                       'p5': np.radians(45), 'p6': np.radians(40)}

        self.pose_folded_grip_right_c1_old = {'p1': np.radians(14), 'p2': np.radians(90),
                                          'p3': np.radians(63), 'p4': np.radians(85),
                                          'p5': np.radians(14), 'p6': np.radians(12)}


        self.pose_folded_grip_right_c1 = {'p1': np.radians(57), 'p2': np.radians(96),
                                          'p3': np.radians(-17), 'p4': np.radians(0),
                                          'p5': np.radians(14), 'p6': np.radians(-90)}



        self.pose_folded_grip_right_c2_old = {'p1': np.radians(20), 'p2': np.radians(85),
                                          'p3': np.radians(90), 'p4': np.radians(89),
                                          'p5': np.radians(11), 'p6': np.radians(15)}


        self.pose_folded_grip_right_c2 = {  'p1': np.radians(86), 'p2': np.radians(80),
                                          'p3': np.radians(-62), 'p4': np.radians(-115),
                                          'p5': np.radians(0), 'p6': np.radians(0)}


        self.pose_folded_grip_right_c3_old = {'p1': np.radians(45), 'p2': np.radians(75),
                                          'p3': np.radians(100), 'p4': np.radians(130),
                                          'p5': np.radians(0), 'p6': np.radians(0)}

        self.pose_folded_grip_right_c4_old = {'p1': np.radians(135), 'p2': np.radians(65),
                                          'p3': np.radians(100), 'p4': np.radians(130),
                                          'p5': np.radians(0), 'p6': np.radians(0)}

        #self.pose_boring_walk_front2 = {'p1': np.radians(-110), 'p2': np.radians(-75),
        #                               'p3': np.radians(147), 'p4': np.radians(90),
        #                               'p5': np.radians(60), 'p6': np.radians(40)}

        self.pose_waiting_arms_side = {'p1': np.radians(-45), 'p2': np.radians(-60),
                                       'p3': np.radians(105), 'p4': np.radians(90),
                                       'p5': np.radians(10), 'p6': np.radians(50)}

        #self.pose_boring_walk_front = {'p1': np.radians(-80), 'p2': np.radians(-60),
        #                               'p3': np.radians(110), 'p4': np.radians(80),
        #         'v1': 0, 'v2': 0, 'v3': 0, 'v4': 0, 'v5': 0, 'v6': 0, 'v7': 0}

        #self.pose_marsh_walk_front = {'p1': np.radians(-100), 'p2': np.radians(-60),
        #                              'p3': np.radians(130), 'p4': np.radians(90)}

        '''
    def movePose(self, pose, duration=4):
        jtp_list_left = list()

        jtp_list_left.append(JTP(rel_time=duration, **pose))
        jtp_list_right = JTP.get_mirrored_jtp_list(jtp_list_left)

        return jtp_list_left, jtp_list_right


    def boring_movement_variant(self):
        back_left =      {'p1': np.radians(35),  'p2': np.radians(-65), 'p3':  np.radians(89),  'p4': np.radians(60)}
        norm_left =      {'p1': np.radians(55),  'p2': np.radians(-58), 'p3':  np.radians(90),  'p4': np.radians(70)}
        norm_front_cp1 = {'p1': np.radians(85),  'p2': np.radians(-54), 'p3': np.radians(130),  'p4': np.radians(74)}
        front_left =     {'p1': np.radians(120), 'p2': np.radians(-65), 'p3': np.radians(160),  'p4': np.radians(75)}

        time_delta = 2
        jtp_list_left = list()
        #jtp_list_left.append(JTP(rel_time=8))
        jtp_list_left.append(JTP(rel_time=6, **norm_left))
        for i in range(4):
            jtp_list_left.append(JTP(rel_time=time_delta, **back_left))
            jtp_list_left.append(JTP(rel_time=time_delta, **norm_left))
            jtp_list_left.append(JTP(rel_time=time_delta, **norm_front_cp1))
            jtp_list_left.append(JTP(rel_time=time_delta, **front_left))
            jtp_list_left.append(JTP(rel_time=time_delta, **norm_front_cp1))
            jtp_list_left.append(JTP(rel_time=time_delta, **norm_left))

        return jtp_list_left, list()

    #todo: rename... nad/or recover old
    def movePoseSplit(self, pose_list_left, pose_list_right, duration=4, times=1):
        assert len(pose_list_left) == len(pose_list_right)

        jtp_list_left = list()
        jtp_list_right = list()

        for i in range(times):
            for idx in range(len(pose_list_left)):
                jtp_list_left.append(JTP(rel_time=duration, **pose_list_left[idx]))
                jtp_list_right.append(JTP(rel_time=duration, **pose_list_right[idx]))


        jtp_list_right = JTP.get_mirrored_jtp_list(jtp_list_right)

        return jtp_list_left, jtp_list_right

    def buildSlenderArms(self, dotime_step, times):

        jtp_flow_data = JTP.extend_base_list(None, *self.movePose(ArmMovement.pose_boring_walk_front_back_c1, dotime_step))

        pose_list_left = [ArmMovement.pose_boring_walk_front_back_c1, ArmMovement.pose_boring_walk_front,
                          ArmMovement.pose_boring_walk_front_back_c1, ArmMovement.pose_boring_walk_back]

        pose_list_right = [ArmMovement.pose_boring_walk_front_back_c1, ArmMovement.pose_boring_walk_back,
                           ArmMovement.pose_boring_walk_front_back_c1, ArmMovement.pose_boring_walk_front]

        jtp_flow_data = JTP.extend_base_list(jtp_flow_data, *self.movePoseSplit(pose_list_left=pose_list_left,
                                                                       pose_list_right=pose_list_right,
                                                                       duration=dotime_step, times=times))

        JTP.extend_base_list(jtp_flow_data, *self.movePose(ArmMovement.pose_boring_walk_front_back_c1, dotime_step))

        return jtp_flow_data


class Bricks(object):
    def __init__(self, profile):
        self.profile = profile

    def calc_samples(self, duration):
        return duration / self.profile.sample_time

    def sin(self, start, stop, duration):
        return np.sin(np.linspace(start, stop, self.calc_samples(duration)))

    def sin_t(self, A, f, phi, T):
        return A * self.sin(phi, f*np.pi*2*T+phi, T)

    def cos(self, start, stop, duration):
        return np.cos(np.linspace(start, stop, self.calc_samples(duration)))

    def lin_acc(self, velocity_start, velocity_lin, velocity_end, duration, acc_percentage=0.2, dec_percentage=0.2):
        TL = self.acc(velocity_start=velocity_start, velocity_end=velocity_lin, duration=duration*acc_percentage)
        TL = np.append(TL, self.lin(velocity=velocity_lin, duration=duration * (1.0-(acc_percentage + dec_percentage))))
        TL = np.append(TL, self.acc(velocity_start=velocity_lin, velocity_end=velocity_end, duration=duration * dec_percentage))
        return TL

    def lin(self, velocity, duration):
        return np.linspace(velocity, velocity, self.calc_samples(duration))

    def lin_dist(self, distance, duration):
        velocity = float(distance) / float(duration)
        print velocity
        return self.lin(velocity, duration)

    def acc(self, velocity_start, velocity_end, duration):
        if velocity_start < velocity_end:
            sin_intv = self.sin(-np.pi/2, np.pi/2, duration)
            velocity_low, velocity_hi = velocity_start, velocity_end
        else:
            sin_intv = self.sin(np.pi/2, np.pi*3/2, duration)
            velocity_low, velocity_hi = velocity_end, velocity_start
        return (sin_intv + 1) / 2 * (velocity_hi - velocity_low) + velocity_low

    def circular_path(self, radius, phi, duration, acc_percentage=0.2, dec_percentage=0.2):

        anz_samples_acc = self.calc_samples(duration * acc_percentage)
        tdata_acc = np.linspace(0, self.profile.sample_time * anz_samples_acc, anz_samples_acc)

        anz_samples_dec = self.calc_samples(duration * dec_percentage)
        tdata_dec = np.linspace(0, self.profile.sample_time * anz_samples_dec, anz_samples_dec)


        th_max = -2.0*phi / (duration * (acc_percentage + dec_percentage - 2.0))
        th_t_acc = th_max * (-np.cos(np.pi * tdata_acc / (duration * acc_percentage)) + 1.0) / 2.0
        th_t_lin = self.lin(velocity=th_max, duration=duration * (1.0-(acc_percentage + dec_percentage)))
        th_t_dec = th_max * (-np.cos(np.pi * tdata_dec / (duration * dec_percentage)) + 1.0) / 2.0

        th_t = np.append(th_t_acc, th_t_lin)
        th_t = np.append(th_t, th_t_dec[::-1])

        v_t = th_t * radius

        return v_t, th_t

    def const_direction_rotation(self, velocity_start_x, velocity_start_y, phi, duration):
        _, tlth = self.circular_path(radius=0, phi=phi, duration=duration, acc_percentage=0.2, dec_percentage=0.2)

        cos_scale = self.cos(start=0, stop=phi, duration=duration)
        sin_scale = self.sin(start=0, stop=phi, duration=duration)

        tlx = cos_scale * velocity_start_x - sin_scale * velocity_start_y
        tly = sin_scale * velocity_start_x + cos_scale * velocity_start_y

        if phi > 0:
            tly *= -1
        if phi < 0:
            tlx *= -1

        return tlx, tly, tlth

class BaseScene(Timeline, Bricks, Bezier, ArmMovement):
    def __init__(self, profile):
            super(BaseScene, self).__init__(profile)

class DummyScene(BaseScene):
    def __init__(self, profile):
            super(DummyScene, self).__init__(profile)

    def not_yet_implemented(self):
        PrettyOutput.attation_msg('scene not yet implemented')


class StuffToTest(BaseScene):
    def __init__(self, profile):
            super(StuffToTest, self).__init__(profile)


    def test_map(self):
        self.new_section('90 Grad Y')
        tlx, tlth = self.circular_path(radius=0.5, phi=np.pi/2, duration=6, acc_percentage=0.35, dec_percentage=0.35)
        self.appendY(tlx)
        self.appendTH(tlth)
        self.syncTimeline()

        self.new_section('LIN X')
        self.appendX(self.lin(duration=2, velocity=0.4))

        self.new_section('90 Grad X')
        tlx, tlth = self.circular_path(radius=0.5, phi=np.pi/2, duration=6, acc_percentage=0.35, dec_percentage=0.35)
        self.appendX(tlx)
        self.appendTH(tlth)
        self.syncTimeline()


    def test_speed_linear(self):
        self.appendX(self.lin(duration=2, velocity=0))
        self.appendX(self.lin_acc(velocity_start=0, velocity_lin=0.7, velocity_end=0, acc_percentage=0.4, dec_percentage=0.4, duration=3))
        self.syncTimeline()

        #self.appendReversePath()

    def test_speed_angular(self):
        self.appendX(self.lin(duration=2, velocity=0))
        self.syncTimeline()

        tlx, tlth = self.circular_path(radius=0, phi=np.pi, duration=1.8, acc_percentage=0.35, dec_percentage=0.35)
        self.appendX(tlx)
        self.appendTH(tlth)

        self.syncTimeline()

        self.appendReversePath()

    def test_speed_circula_path(self):
        self.appendX(self.lin(duration=3, velocity=0))
        self.syncTimeline()

        tlx, tlth = self.circular_path(radius=5, phi=np.pi/2, duration=3, acc_percentage=0.2, dec_percentage=0.2)
        self.appendX(tlx)
        self.appendTH(tlth)

        self.syncTimeline()

        self.appendReversePath()

    def test_stuff(self):
        self.appendX(self.lin(duration=2, velocity=0))

        # KREISBAHN
        ############

        self.new_section('test lin')
        self.appendX(self.lin_acc(velocity_start=0, velocity_lin=0.2, velocity_end=0, acc_percentage=0.2, dec_percentage=0, duration=2))
        self.appendX(self.lin_acc(velocity_start=0.2, velocity_lin=0.4, velocity_end=0, acc_percentage=0.2, dec_percentage=0.35, duration=2))


        self.new_section('test circular_path')
        tlx, tlth = self.circular_path(radius=0.5, phi=np.pi/2, duration=3.8, acc_percentage=0.3, dec_percentage=0.3)
        self.appendX(tlx)
        self.appendTH(tlth)

        self.syncTimeline()


    def test_slender_arms(self):

        self.appendArms(self.movePose(duration=6, pose=ArmMovement.pose_home))
        self.appendArms(self.movePose(duration=6, pose=ArmMovement.pose_boring_walk_front_back_c1))

        self.appendArms(self.buildSlenderArms(dotime_step=1.75, times=2))

        #self.appendArms(self.movePose(duration=6, pose=ArmMovement.pose_boring_walk_front_back_c1))
        self.appendArms(self.movePose(duration=6, pose=ArmMovement.pose_home))

    def test_run_arms(self):
        #todo: new class etc...
        #self.appendArms(self.movePose(duration=8, pose=ArmMovement.pose_home))


        self.appendArms(self.movePose(duration=3, pose=ArmMovement.pose_run_arms))
        self.appendArms(self.movePose(duration=1, pose=ArmMovement.pose_run_arms))


        self.syncTimeline()
        #return

        self.appendX(self.lin_acc(velocity_start=0, velocity_lin=0.6, velocity_end=0, duration=9, acc_percentage=0.2, dec_percentage=0.2))

        dotime = 1.25
        sin = self.sin(0, np.pi/2, dotime/4.0)
        cos = self.cos(0, np.pi*2, dotime)
        ntimes = 9

        j1speed = 0.5
        j4speed = 0.5
        j5speed = 0.5
        j6speed = 0.5

        self.appendVelArmLeft(j1=sin*-j1speed, j4=sin*j4speed, j5=sin*j5speed, j6=sin*j6speed)
        for i in range(ntimes):
            self.appendVelArmLeft(j1=cos*-j1speed, j4=cos*j4speed, j5=cos*j5speed, j6=cos*j6speed)
        sin = sin[::-1]
        self.appendVelArmLeft(j1=sin*-j1speed, j4=sin*j4speed, j5=sin*j5speed, j6=sin*j6speed)

        sin = sin[::-1]
        #sin *= -1
        #cos *= -1

        self.appendVelArmRight(j1=sin*-j1speed, j4=sin*j4speed, j5=sin*j5speed, j6=sin*j6speed)
        for i in range(ntimes):
            self.appendVelArmRight(j1=cos*-j1speed, j4=cos*j4speed, j5=cos*j5speed, j6=cos*j6speed)
        sin = sin[::-1]
        self.appendVelArmRight(j1=sin*-j1speed, j4=sin*j4speed, j5=sin*j5speed, j6=sin*j6speed)

        self.appendSwitchVelToGoalTimeout()

        self.syncTimeline()

    def testBezier(self):
        #points = [[0,0], [0,1], [1,1], [1.5, 0.5], [2, 0.5], [2, 0], [1, 0.5], [1, 0],
        #                   [1, -1], [1, -1], [1, -1], [2, -1], [3, -1], [3, -1], [3, -1], [3, 0],
        #                   [2.5, 0.5], [2, 1], [1.5, 1], [0.5, 1], [-0.5, 1], [-0.5, 0], [-0.5, -1], [0.5, -1], ]
        #points = [[0,0], [0,0.5], [0.5,0.5]]
        yscale = 0.25
        xscale = 0.25
        points = [[0, 0], [2, -1], [3, 0], [4, 1], [5.5, 0], [6, 0]]
        points = np.array(points, np.float)
        points[..., 1] *= yscale
        points[..., 0] *= xscale
        print

        x, th = self.createBezier(points, duration=8)

        self.appendX(self.acc(velocity_start=0, velocity_end=x[0], duration=1))
        self.syncTimeline()

        self.appendX(x)
        self.appendTH(th)

        self.appendX(self.acc(velocity_start=x[-1], velocity_end=0, duration=1))


class BoringScene_1_2_3(BaseScene):
    def __init__(self, profile):
        super(BoringScene_1_2_3, self).__init__(profile)

    def bridge_act_1_arms_startpos(self, dotime=8):
        self.appendArms(self.movePose(duration=dotime, pose=ArmMovement.pose_boring_walk_front_back_c1))
        self.syncTimeline()

    def act_1_slender_around(self):
        dotime = 10.5
        linspeed = 0.3
        steps = 2
        steps = 1
        #dotime = 3.5 * (steps+1)


        stepduration = dotime / (steps + 1.0)

        sin = self.sin(0, np.pi/2, stepduration/4.0)
        sinrev = sin[::-1]
        cos = self.cos(0, np.pi*2, stepduration)

        thspeed = 0.4
        sin *= thspeed
        cos *= thspeed

        ioscale = 0.55


        self.appendX(self.acc(velocity_start=0, velocity_end=linspeed, duration=stepduration/2.0))
        self.syncTimeline()

        self.appendArms(self.buildSlenderArms(dotime_step=1.75, times=steps-1))

        self.appendX(self.lin(velocity=linspeed, duration=stepduration/2.0))

        self.appendTH(sin*-ioscale)
        self.appendTH(sinrev*-ioscale)

        self.appendTH(sin)
        self.appendX(self.lin(velocity=linspeed, duration=stepduration))

        for i in range(steps):
            direction = 1 if (i % 2) else -1
            self.appendX(self.lin(velocity=linspeed, duration=stepduration))
            self.appendTH(cos)

        direction = 1 if (steps % 2) else -1
        self.appendTH(sinrev)


        #self.appendX(self.lin(velocity=linspeed, duration=stepduration/2.0))
        self.appendTH(sin*-ioscale)
        self.appendTH(sinrev*-ioscale)


        self.appendX(self.acc(velocity_start=linspeed, velocity_end=0, duration=stepduration/2.0))


        self.syncTimeline()

    def slender_around_old(self):
        dotime = 10.5
        linspeed = 0.3
        steps = 2
        steps = 3
        dotime = 3.5 * (steps+1)


        stepduration = dotime / (steps + 1.0)
        print stepduration

        accp = 0.5
        decp = 0.5
        _, tlth_quater = self.circular_path(radius=1, phi=np.pi/4, duration=stepduration/2.0, acc_percentage=accp, dec_percentage=decp)
        _, tlth_half = self.circular_path(radius=1, phi=np.pi/2, duration=stepduration, acc_percentage=accp, dec_percentage=decp)


        self.appendX(self.acc(velocity_start=0, velocity_end=linspeed, duration=stepduration/2.0))
        self.syncTimeline()

        self.appendArms(self.buildSlenderArms(dotime_step=1.75, times=steps-1))

        self.appendX(self.lin(velocity=linspeed, duration=stepduration/2.0))
        self.appendTH(tlth_quater)

        for i in range(steps):
            direction = 1 if (i % 2) else -1
            self.appendX(self.lin(velocity=linspeed, duration=stepduration))
            self.appendTH(tlth_half*direction)

        self.appendX(self.lin(velocity=linspeed, duration=stepduration/2.0))
        direction = 1 if (steps % 2) else -1
        self.appendTH(tlth_quater*direction)

        self.appendX(self.acc(velocity_start=linspeed, velocity_end=0, duration=stepduration/2.0))



        self.syncTimeline()


    def bridge_act_2_arms_startpos(self, dotime=8):
        self.appendArms(self.movePose(duration=dotime, pose=ArmMovement.bridge_pose_boring_walk_c1_to_waiting_arms_side))
        #self.appendArms(self.movePose(duration=dotime, pose=ArmMovement.pose_waiting_arms_side))
        self.syncTimeline()

    def act_2_1_to_window(self):
        self.syncTimeline()

        self.new_section('annaehern')

        self.appendX(self.acc(velocity_start=0, velocity_end=0.21, duration=0.5))
        self.appendX(self.lin(0.21, 2))

        self.new_section('ausrichtung fenster')
        tlx, tlth = self.circular_path(radius=-1, phi=-np.pi/2, duration=10, acc_percentage=0, dec_percentage=0.5)
        self.appendX(tlx)
        self.appendTH(tlth)

        #TODO: am ende von den 10 sek
        self.syncTimeline()
        self.appendArms(self.movePose(duration=2, pose=ArmMovement.pose_waiting_arms_side))

        self.new_section('look')
        self.appendX(self.lin(duration=0.25, velocity=0))
        self.syncTimeline()

        tlx, tlth = self.circular_path(radius=0, phi=-np.pi/4, duration=3.5, acc_percentage=0.6, dec_percentage=0.4)
        self.appendX(tlx)
        self.appendTH(tlth)
        self.syncTimeline()

        self.appendX(self.lin(duration=1, velocity=0))
        self.syncTimeline()

        tlx, tlth = self.circular_path(radius=0, phi=np.pi/2, duration=5, acc_percentage=0.4, dec_percentage=0.5)
        self.appendX(tlx)
        self.appendTH(tlth)
        self.syncTimeline()

        self.appendX(self.lin(duration=1, velocity=0))
        self.syncTimeline()


    def act_2_2_away_from_window(self):

        self.new_section('\nenfernen vom fenster')

        dotime = 2
        self.appendArms(self.movePose(duration=dotime, pose=ArmMovement.bridge_pose_boring_walk_c1_to_waiting_arms_side))
        self.appendArms(self.movePose(duration=dotime, pose=ArmMovement.pose_boring_walk_front_back_c1))

        self.appendY(self.lin(velocity=0, duration=0.5))
        self.appendY(self.lin_acc(velocity_start=0, velocity_lin=0.15, velocity_end=0, acc_percentage=0.3, dec_percentage=0.3, duration=3.5))

        self.appendTH(self.lin_acc(velocity_start=0, velocity_lin=0.55, velocity_end=0, acc_percentage=0.35, dec_percentage=0.25, duration=5))

        self.appendX(self.lin_acc(velocity_start=0, velocity_lin=-0.08, velocity_end=0, acc_percentage=0.7, dec_percentage=0.3, duration=1.5))
        self.appendX(self.lin_acc(velocity_start=0, velocity_lin=0.2, velocity_end=0, acc_percentage=0.15, dec_percentage=0.2, duration=5.5))

        self.syncTimeline()


    def bridge_act_3_arms_startpos(self, dotime=8):
        self.appendArms(self.movePose(duration=dotime, pose=ArmMovement.pose_boring_walk_front_back_c1))
        self.syncTimeline()

    def act_3_move_corner_shock(self):
        self.syncTimeline()

        steps = 3

        self.appendArms(self.buildSlenderArms(dotime_step=1.75, times=steps-1))



        self.syncTimeline()


class RunAwayScene_4(BaseScene):
    def __init__(self, profile):
        super(RunAwayScene_4, self).__init__(profile)

    def bridge_act_4_arms_startpos(self, dotime=8):
        self.appendArms(self.movePose(duration=dotime, pose=ArmMovement.pose_run_arms))
        self.syncTimeline()


    def act_4_run_away(self, dotime=9):
        self.syncTimeline()

        self.appendArms(self.movePose(duration=1, pose=ArmMovement.pose_run_arms))

        self.syncTimeline()

        self.appendX(self.lin_acc(velocity_start=0, velocity_lin=0.6, velocity_end=0, duration=dotime,
                                  acc_percentage=0.2, dec_percentage=0.2))

        dotime_step = 1.25
        ntimes = int(np.ceil(dotime / dotime_step))

        sin = self.sin(0, np.pi/2, dotime_step/4.0)
        sinrev = sin[::-1]
        cos = self.cos(0, np.pi*2, dotime_step)

        j1speed = 0.5
        j4speed = 0.5
        j5speed = 0.5
        j6speed = 0.5

        self.appendVelArmLeft(j1=sin*-j1speed, j4=sin*j4speed, j5=sin*j5speed, j6=sin*j6speed)
        self.appendVelArmRight(j1=sin*-j1speed, j4=sin*j4speed, j5=sin*j5speed, j6=sin*j6speed)

        for i in range(ntimes):
            self.appendVelArmLeft(j1=cos*-j1speed, j4=cos*j4speed, j5=cos*j5speed, j6=cos*j6speed)
            self.appendVelArmRight(j1=cos*-j1speed, j4=cos*j4speed, j5=cos*j5speed, j6=cos*j6speed)

        self.appendVelArmLeft(j1=sinrev*-j1speed, j4=sinrev*j4speed, j5=sinrev*j5speed, j6=sinrev*j6speed)
        self.appendVelArmRight(j1=sinrev*-j1speed, j4=sinrev*j4speed, j5=sinrev*j5speed, j6=sinrev*j6speed)

        self.syncTimeline()
        self.appendSwitchVelToGoalTimeout()
        self.syncTimeline()

        self.appendArms(self.movePose(duration=1, pose=ArmMovement.pose_run_arms))


class FindingRose_5(BaseScene):
    def __init__(self, profile):
        super(FindingRose_5, self).__init__(profile)

    def bridge_act_5_arm_right_startpos(self, dotime=8):
        jtp_flow_data = [list(), list()]
        jtp_flow_data[1].append(JTP(rel_time=dotime, **ArmMovement.pose_right_folded_back))
        self.appendArms(jtp_flow_data, True)
        self.syncTimeline()

    def act_5_1_griper_to_rose(self):
        jtp_flow_data = [list(), list()]

        jtp_flow_data[1].append(JTP(rel_time=1.5, **ArmMovement.pose_folded_grip_right_c1))
        jtp_flow_data[1].append(JTP.get_mirrored_jtp(JTP(rel_time=2, **ArmMovement.pose_boring_walk_back)))
        jtp_flow_data[1].append(JTP(rel_time=1.5, **ArmMovement.pose_folded_grip_right_c2))
        jtp_flow_data[1].append(JTP(rel_time=1.5, **ArmMovement.pose_folded_grip_right_c3))
        jtp_flow_data[1].append(JTP(rel_time=1.25, **ArmMovement.pose_folded_grip_right_c4))

        pargs = dict(ArmMovement.pose_grip_rose_right)
        pargs.update({'v1': 0, 'v2': 0, 'v3': 0, 'v4': 0, 'v5': 0, 'v6': 0, 'v7': 0})
        jtp_flow_data[1].append(JTP(rel_time=2, **pargs))

        self.appendArms(jtp_flow_data, True)
        self.syncTimeline()


    def act_5_2_grip_rose(self):
        pass

    def act_5_3_gripper_away_from_rose(self):
        jtp_flow_data = [list(), list()]

        jtp_flow_data[1].append(JTP(rel_time=8, **ArmMovement.pose_folded_grip_right_c6))
        jtp_flow_data[1].append(JTP(rel_time=1.25, **ArmMovement.pose_folded_grip_right_c4))

        pargs = dict(ArmMovement.pose_carry_rose_front_right)
        pargs.update({'v1': 0, 'v2': 0, 'v3': 0, 'v4': 0, 'v5': 0, 'v6': 0, 'v7': 0})
        jtp_flow_data[1].append(JTP(rel_time=2, **pargs))

        self.appendArms(jtp_flow_data, True)
        self.syncTimeline()

    def act_5_4_drive_away(self):
        pass

class CheeringScene_7_8_9_10(BaseScene):
    def __init__(self, profile):
        super(CheeringScene_7_8_9_10, self).__init__(profile)

    def bridge_act_7_arms_startpos(self, dotime=8):
        self.appendArms(self.movePose(duration=dotime, pose=ArmMovement.pose_cheer_arms_up))
        self.syncTimeline()

    def act_7_cheer_arms_up(self, lin_speed=0.5, dotime=1, ntimes=5):

        self.syncTimeline()

        self.appendX(self.lin_acc(velocity_start=0, velocity_lin=lin_speed, velocity_end=0, duration=10))

        dotime = 1

        zero = self.lin(0, dotime)
        cos_0_pi = self.cos(0, np.pi, dotime)
        cos_pi_2pi = self.cos(np.pi, np.pi*2, dotime)
        sin_0_2pi = self.sin(0, np.pi*2, dotime)

        jj = [
                [ # j1
                    [sin_0_2pi, sin_0_2pi],  # j1 signal
                    [-0.3, 0.3],  # j1 left and right speed
                ],
                [ # j2
                    [zero, zero],
                    [0, 0],  # j2 left and right speed
                ],
                [ # j3
                    [cos_0_pi, cos_pi_2pi],
                    [0.4, 0.4],
                ],
                [ # j4
                    [sin_0_2pi, sin_0_2pi],
                    #[zero, zero],
                    [-0.4, 0.4],
                ],
                [ # j5
                    [zero, zero],
                    [0, 0],
                ],
                [ # j6
                    [zero, zero],
                    [0, 0],
                ],
                [ # j7
                    [zero, zero],
                    [0, 0],
                ],
        ]

        jointlist = list()
        for jn in range(7):
            step_data, velocities = zip(*zip(*jj[jn]))
            jointlist.append(map(lambda step: [step, velocities[0], velocities[1]], step_data))

        steplist = zip(*jointlist)


        for _ in range(ntimes):
            for step in steplist:
                joint_names = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'j7']
                joint_data, left_velocity, right_velocity = zip(*step)
                joint_data_left = map(op.mul, joint_data, left_velocity)
                joint_data_right = map(op.mul, joint_data, right_velocity)

                print dict(zip(joint_names, joint_data_left))

                self.appendVelArmLeft(**dict(zip(joint_names, joint_data_left)))
                self.appendVelArmRight(**dict(zip(joint_names, joint_data_right)))

        self.syncTimeline()

        self.appendVelArm(j1=np.array([0], np.float64))

    def bridge_act_8_arms_startpos(self, dotime=8):
        self.appendArms(self.movePose(duration=dotime, pose=ArmMovement.pose_cheer_arms_up))

        self.syncTimeline()


    def act_8_cheering_turn(self, lin_speed=0.55, acctime=1.5, phi=np.pi*2, rot_duration=8, arm_duration=2.75):

        arm_sleep = rot_duration - arm_duration * 2
        assert arm_sleep >= 0

        arm_sleep_samples = int(self.calc_samples(arm_sleep))


        self.syncTimeline()

        self.appendX(self.acc(velocity_start=0, velocity_end=lin_speed, duration=acctime))

        self.syncTimeline()

        self.appendArms(self.movePose(duration=arm_duration, pose=ArmMovement.pose_cheer_turn))

        tlx, tly, tlth = self.const_direction_rotation(velocity_start_x=lin_speed, velocity_start_y=0, phi=phi, duration=rot_duration)
        self.appendX(tlx)
        self.appendY(tly)
        self.appendTH(tlth)

        fill_data = [None]*arm_sleep_samples
        self.ARML_GOAL.extend(fill_data)
        self.ARMR_GOAL.extend(fill_data)

        self.appendArms(self.movePose(duration=arm_duration, pose=ArmMovement.pose_cheer_arms_up))


        self.appendX(self.acc(velocity_start=lin_speed, velocity_end=0, duration=acctime))

        self.syncTimeline()


    def bridge_act_9_arms_startpos(self, dotime=8):
        self.appendArms(self.movePose(duration=dotime, pose=ArmMovement.pose_run_arms))


    def act_9_drumming_rotmove_side_drive(self, lin_speed=0.25, acc_duration=1, rot_duration=2, ndrums=1,
                                pre_lin_duration=2, post_lin_duration=2):


        # SETUP
        ########

        phi = np.pi / 2
        goto_drum_pose_duration = 3.5

        pre_ld1 = pre_lin_duration - (goto_drum_pose_duration - rot_duration)
        pre_ld2 = goto_drum_pose_duration - rot_duration

        assert pre_ld1 >= 0
        assert pre_ld2 >= 0
        assert goto_drum_pose_duration <= (rot_duration + pre_lin_duration)

        dotime_step = 1.25

        lin_duration = ndrums * dotime_step + dotime_step / 2

        sin = self.sin(0, np.pi/2, dotime_step/4.0)
        sinrev = sin[::-1]
        cos = self.cos(0, np.pi*2, dotime_step)

        j1speed = 0.5
        j4speed = 0.5
        j5speed = 0.5
        j6speed = 0.5


        # MOVEMENT
        ###########

        self.syncTimeline()

        self.appendX(self.acc(velocity_start=0, velocity_end=lin_speed, duration=acc_duration))


        self.syncTimeline()

        self.appendX(self.lin(lin_speed, pre_ld1))

        self.syncTimeline()

        self.appendX(self.lin(lin_speed, pre_ld2))
        self.syncTimelineBase()

        pargs = dict(ArmMovement.pose_cheer_drum)
        pargs.update({'v1': 0, 'v2': 0, 'v3': 0, 'v4': 0, 'v5': 0, 'v6': 0, 'v7': 0})
        self.appendArms(self.movePose(duration=goto_drum_pose_duration, pose=pargs))

        tlx, tly, tlth = self.const_direction_rotation(velocity_start_x=lin_speed, velocity_start_y=0, phi=phi, duration=rot_duration)
        self.appendX(tlx)
        self.appendY(tly)
        self.appendTH(tlth)

        self.syncTimeline()

        self.appendY(self.lin(-lin_speed, lin_duration))

        self.appendVelArmLeft(j1=sin*-j1speed, j4=sin*j4speed, j5=sin*j5speed, j6=sin*j6speed)
        for _ in range(ndrums):
            self.appendVelArmLeft(j1=cos*-j1speed, j4=cos*j4speed, j5=cos*j5speed, j6=cos*j6speed)
        self.appendVelArmLeft(j1=sinrev*-j1speed, j4=sinrev*j4speed, j5=sinrev*j5speed, j6=sinrev*j6speed)

        #sin *= -1
        #sinrev *= -1
        #cos *= -1
        self.appendVelArmRight(j1=sin*-j1speed, j4=sin*j4speed, j5=sin*j5speed, j6=sin*j6speed)
        for _ in range(ndrums):
            self.appendVelArmRight(j1=cos*-j1speed, j4=cos*j4speed, j5=cos*j5speed, j6=cos*j6speed)
        self.appendVelArmRight(j1=sinrev*-j1speed, j4=sinrev*j4speed, j5=sinrev*j5speed, j6=sinrev*j6speed)



        self.syncTimeline()

        self.appendSwitchVelToGoalTimeout()
        self.syncTimelineArmGoal()

        tlx, tly, tlth = self.const_direction_rotation(velocity_start_x=0, velocity_start_y=-lin_speed, phi=-phi, duration=rot_duration)
        self.appendX(tlx)
        self.appendY(tly)
        self.appendTH(tlth)

        self.appendArms(self.movePose(duration=goto_drum_pose_duration, pose=ArmMovement.pose_run_arms))

        self.appendX(self.lin(lin_speed, post_lin_duration))

        self.appendX(self.acc(velocity_start=lin_speed, velocity_end=0, duration=acc_duration))

    def act_10_corner_rotation(self, duration=16):

        points = [[0,0], [4,0], [1,2]]
        points = np.array(points, np.float)
        x, th = self.createBezier(points, duration=duration)

        self.appendX(self.acc(velocity_start=0, velocity_end=x[0], duration=1))
        self.syncTimeline()


        _, tlth = self.circular_path(radius=0, phi=-np.pi*2, duration=duration, acc_percentage=0.2, dec_percentage=0.2)
        tlth = tlth[1:-1]
        thover = th + tlth

        absphi = integrate.cumtrapz(tlth*self.profile.sample_time, initial=0)
        absphi = np.append(absphi, np.array([0]))

        absphi *= np.pi*2/np.abs(absphi).max()

        xvel = x * np.cos(absphi)
        yvel = x * -np.sin(absphi)

        self.appendX(xvel)
        self.appendY(yvel)
        self.appendTH(thover)

        self.appendX(self.acc(velocity_start=x[-1], velocity_end=0, duration=1))


if __name__ == '__main__':

    cob3_3_profile = Profile(rate=100, max_linear_velocity=0.7, max_angular_velocity=2.7,
                             max_linear_acceleration=0.022, max_angular_acceleration=0.074, switch_vel_to_goal_timeout=0.1)

    cob4_2_profile = Profile(rate=30, max_linear_velocity=0.7, max_angular_velocity=2.7,
                             max_linear_acceleration=0.022, max_angular_acceleration=0.074, switch_vel_to_goal_timeout=0.7  )

    boring = BoringScene_1_2_3(profile=cob4_2_profile)
    cheer = CheeringScene_7_8_9_10(profile=cob4_2_profile)
    findrose = FindingRose_5(profile=cob4_2_profile)
    test = StuffToTest(profile=cob4_2_profile)
    discover = RunAwayScene_4(profile=cob4_2_profile)
    dummy = DummyScene(profile=cob4_2_profile)

    #boring.bridge_act_1_arms_startpos()
    boring.act_1_slender_around()
    #boring.bridge_act_2_arms_startpos()
    #boring.act_2_1_to_window()
    #boring.act_2_2_away_from_window()


    #cheer.bridge_act_7_arms_startpos()
    #cheer.act_7_cheer_arms_up()
    #cheer.act_8_cheering_turn()
    #cheer.act_9_drumming_rotmove_side_drive()
    cheer.act_10_corner_rotation()
    #test.test_map()
    #test.test_speed_linear()
    #test.test_speed_angular()
    #test.test_speed_circula_path()


    #test.test_slender_arms()  # ok
    #test.test_run_arms()  # ok
    #test.test_cheer_arms()  # (ok)

    #test.test_rotmove_side_drive()

    #test.testBezier()




    #boring.appendReversePath()
    #test.appendReversePath()

    #test.testGripRose()


    # SETTING MASTER TIMELINE
    ##########################

    #masterTimeline = test
    #masterTimeline = boring
    masterTimeline = cheer

    sh = ServiceHandler()
    sh.add_service_callback('scenario/br1', boring.bridge_act_1_arms_startpos, boring)
    sh.add_service_callback('scenario/sc1', boring.act_1_slender_around, boring)
    sh.add_service_callback('scenario/br2', boring.bridge_act_2_arms_startpos, boring)
    sh.add_service_callback('scenario/sc2', [boring.act_2_1_to_window, boring.act_2_2_away_from_window], boring)
    sh.add_service_callback('scenario/br3', boring.bridge_act_3_arms_startpos, boring)
    sh.add_service_callback('scenario/sc3', boring.act_3_move_corner_shock, boring)
    sh.add_service_callback('scenario/br4', discover.bridge_act_4_arms_startpos, discover)
    sh.add_service_callback('scenario/sc4', discover.act_4_run_away, discover)
    sh.add_service_callback('scenario/br5', findrose.bridge_act_5_arm_right_startpos, findrose)
    sh.add_service_callback('scenario/sc5', [findrose.act_5_1_griper_to_rose, findrose.act_5_2_grip_rose,
                                             findrose.act_5_3_gripper_away_from_rose, findrose.act_5_4_drive_away], findrose)
    sh.add_service_callback('scenario/br6', dummy.not_yet_implemented, dummy)
    sh.add_service_callback('scenario/sc6', dummy.not_yet_implemented, dummy)

    sh.add_service_callback('scenario/br7', cheer.bridge_act_7_arms_startpos, cheer)
    sh.add_service_callback('scenario/sc7', cheer.act_7_cheer_arms_up, cheer)
    sh.add_service_callback('scenario/br8', cheer.bridge_act_8_arms_startpos, cheer)
    sh.add_service_callback('scenario/sc8', cheer.act_8_cheering_turn, cheer)
    sh.add_service_callback('scenario/br9', cheer.bridge_act_9_arms_startpos, cheer)
    sh.add_service_callback('scenario/sc9', cheer.act_9_drumming_rotmove_side_drive, cheer)
    sh.add_service_callback('scenario/br10', dummy.not_yet_implemented, dummy)
    sh.add_service_callback('scenario/sc10', cheer.act_10_corner_rotation, cheer)

    sh.add_service_callback('scenario/br11', dummy.not_yet_implemented, dummy)
    sh.add_service_callback('scenario/sc11', dummy.not_yet_implemented, dummy)
    #sh.add_service_callback('scenario/test1', test.test_rotmove_side_drive, test)

    PrettyOutput.attation_msg('APPLICATION STARTED')

    if not sh.is_service_mode:
        sh.execute_timeline(masterTimeline)
    else:
        sh.do_listen()