#!/usr/bin/env python2
#-*- coding: utf-8 -*-

# ROS IMPORTS
import roslib

try:
    isLive=True
    roslib.load_manifest('cobvid')
    print 'cobvid found'
    from cob_srvs.srv import Trigger
    print 'trigger srv found'
    #from cob_mimic.srv import SetMimic, SetMimicRequest
    #print 'mimic srv stuff found'
    #from cob_mimic.msg import SetMimicAction, SetMimicGoal
    #print 'mimic msg stuff found'
    from cob_light.srv import SetLightMode, SetLightModeRequest
    print 'led stuff found'
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
import tf

# ROS MSGS

from geometry_msgs.msg import Twist
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from actionlib_msgs.msg import GoalStatus

# IMPORT FOR CALCULATIONS
import numpy as np

from scipy import integrate
from scipy import interpolate
from scipy.misc import comb

# PYTHON IMPORTS
import sys
import os
import string
import operator as op
import yaml

class DummyObject(object):
    def __init__(self, *args, **kwargs):
        pass

    def __getattribute__(self, name):
        return DummyObject()

if not isLive:
    SetMimicRequest = DummyObject
    SetLightModeRequest = DummyObject

class PrettyOutput(object):

    @staticmethod
    def init_exit_msg(module_name):
        if PrettyOutput.attation_msg('CAN NOT INITIALIZE %s' % module_name, 'EXIT TASK?'):
            ROSBridge._exit_task()

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

        collection = collections.BrokenBarHCollection.span_where(self.tdata, ymin=-self.timeline.profile.max_linear_acceleration, ymax=self.timeline.profile.max_linear_acceleration, where=accwarn1, facecolor='blue', alpha=0.2)
        ax3.add_collection(collection)
        collection = collections.BrokenBarHCollection.span_where(self.tdata, ymin=-self.timeline.profile.max_linear_acceleration, ymax=self.timeline.profile.max_linear_acceleration, where=accwarn2, facecolor='green', alpha=0.2)
        ax3.add_collection(collection)
        collection = collections.BrokenBarHCollection.span_where(self.tdata, ymin=-self.timeline.profile.max_angular_acceleration, ymax=self.timeline.profile.max_angular_acceleration, where=accwarn3, facecolor='red', alpha=0.2)
        ax3.add_collection(collection)

        plt.tight_layout()

    def plot_map(self):
        import matplotlib.pyplot as plt

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

        marker_style = dict(alpha=0.4, markersize=15)


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

        for i in range(len(x))[::int(self.timeline.profile.rate)]:
            text = '\n%s' % int(np.round(self.tdata[i], 0))
            ax1.text(x[i], y[i], text, fontsize=10, alpha=0.40)
            ax1.plot(x[i], y[i], '.', color=(0, 0, 0), alpha=0.20)


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

        def param_call(self, *args):
            print 'FAKE: %s:   %s' % (self.topic_name, args)

    def __init__(self, fakerun=False, exec_base=False,
                 exec_arm_left=False, exec_arm_right=False,
                 exec_gripper_left=False, exec_gripper_right=False,
                 exec_mimic=False, exec_led=False, exec_tf=False, timeout=5):
        self.exec_base = exec_base
        self.exec_arm_left = exec_arm_left
        self.exec_arm_right = exec_arm_right
        self.exec_gripper_left = exec_gripper_left
        self.exec_gripper_right = exec_gripper_right
        self.exec_mimic = exec_mimic
        self.exec_led = exec_led
        self.exec_tf = exec_tf

        BASE_CONTROLLER_TOPIC = '/base_controller/command_direct'

        ARM_LEFT_VELOCITY_TOPIC = '/arm_left/joint_group_velocity_controller/command'
        ARM_LEFT_TRAJECTORY_TOPIC = '/arm_left/joint_trajectory_controller/follow_joint_trajectory'

        ARM_RIGHT_VELOCITY_TOPIC = '/arm_right/joint_group_velocity_controller/command'
        ARM_RIGHT_TRAJECTORY_TOPIC = '/arm_right/joint_trajectory_controller/follow_joint_trajectory'

        LED_TORSO_SERVICE_TOPIC = 'light_torso/mode'
        LED_BASE_SERVICE_TOPIC = 'light_base/mode'

        GRIPPER_LEFT_TRAJECTORY_TOPIC = '/gripper_left/joint_trajectory_controller/follow_joint_trajectory'
        GRIPPER_RIGHT_TRAJECTORY_TOPIC = '/gripper_right/joint_trajectory_controller/follow_joint_trajectory'

        MIMIC_ACTION_TOPIC = '/set_mimic_action'


        queue_size = 0

        # BASE
        self.base_publisher = rospy.Publisher(BASE_CONTROLLER_TOPIC, Twist, queue_size=queue_size) \
            if not fakerun and isLive and exec_base else ROSBridge.Dummy('BASE')

        # LEFT ARM VELOCITY
        self.arm_left_velocity_publisher = rospy.Publisher(ARM_LEFT_VELOCITY_TOPIC, Float64MultiArray, queue_size=queue_size) \
            if not fakerun and isLive and exec_arm_left else ROSBridge.Dummy('ARM_LEFT')

        # RIGHT ARM VELOCITY
        self.arm_right_velocity_publisher = rospy.Publisher(ARM_RIGHT_VELOCITY_TOPIC, Float64MultiArray, queue_size=queue_size) \
            if not fakerun and isLive and exec_arm_right else ROSBridge.Dummy('ARM_RIGHT')

        # MIMIC
        if not fakerun and isLive and exec_mimic:

            self.mimic_action_call = actionlib.SimpleActionClient(MIMIC_ACTION_TOPIC, SetMimicAction)
            if not self.mimic_action_call.wait_for_server(timeout=rospy.Duration(timeout)):
                PrettyOutput.init_exit_msg('MIMIC')

            """
            try:
                rospy.wait_for_service(MIMIC_SERVICE_TOPIC, timeout=timeout)
            except rospy.ROSException:
                PrettyOutput.init_exit_msg('MIMIC')

            self.mimic_call = rospy.ServiceProxy(MIMIC_SERVICE_TOPIC, SetMimic)
            """
        else:
            self.mimic_call = ROSBridge.Dummy('MIMIC').param_call

        # LEDs
        if not fakerun and isLive and exec_led:
            try:
                rospy.wait_for_service(LED_TORSO_SERVICE_TOPIC, timeout=timeout)
            except rospy.ROSException:
                PrettyOutput.init_exit_msg('LEDs')

            self.led_torso_call = rospy.ServiceProxy(LED_TORSO_SERVICE_TOPIC, SetLightMode)
        else:
            self.led_torso_call = ROSBridge.Dummy('LED').param_call


        if not fakerun and isLive:

            ##################################
            # ARM LEFT SYNCHRONOUS TRAJECTORY
            ##################################

            joint_names = [pattern % ('left', idx+1) for idx, pattern in
                           enumerate(['arm_%s_%d_joint']*7)]

            params = {'logic_name': 'ARM LEFT GOAL',
                      'joint_trajectory_topic': ARM_LEFT_TRAJECTORY_TOPIC,
                      'joint_names': joint_names, 'do_init': exec_arm_left,
                      'timeout': timeout}

            self.new_st_arm_left = NewSynchronousTrajectory(**params)

            ###################################
            # ARM RIGHT SYNCHRONOUS TRAJECTORY
            ###################################

            joint_names = [pattern % ('right', idx+1) for idx, pattern in
                           enumerate(['arm_%s_%d_joint']*7)]

            params = {'logic_name': 'ARM RIGHT GOAL',
                      'joint_trajectory_topic': ARM_RIGHT_TRAJECTORY_TOPIC,
                      'joint_names': joint_names, 'do_init': exec_arm_right,
                      'timeout': timeout}

            self.new_st_arm_right = NewSynchronousTrajectory(**params)

            ######################################
            # GRIPPER LEFT SYNCHRONOUS TRAJECTORY
            ######################################

            gripper_joint_pattern = ['gripper_%s_proximal', 'gripper_%s_distal']
            direction = 'left'
            joint_names = [joint_name % direction for joint_name in gripper_joint_pattern]

            params = {'logic_name': 'GRIPPER LEFT',
                      'joint_trajectory_topic': GRIPPER_LEFT_TRAJECTORY_TOPIC,
                      'joint_names': joint_names, 'do_init': exec_gripper_left,
                      'timeout': timeout}

            self.new_st_gripper_left = NewSynchronousTrajectory(**params)

            #######################################
            # GRIPPER RIGHT SYNCHRONOUS TRAJECTORY
            #######################################

            direction = 'right'
            joint_names = [joint_name % direction for joint_name in gripper_joint_pattern]

            params = {'logic_name': 'GRIPPER LEFT',
                      'joint_trajectory_topic': GRIPPER_RIGHT_TRAJECTORY_TOPIC,
                      'joint_names': joint_names, 'do_init': exec_gripper_right,
                      'timeout': timeout}

            self.new_st_gripper_right = NewSynchronousTrajectory(**params)
        else:
            self.new_st_arm_left = ROSBridge.Dummy('ARM LEFT GOAL')
            self.new_st_arm_right = ROSBridge.Dummy('ARM RIGHT GOAL')
            self.new_st_gripper_left = ROSBridge.Dummy('GRIPPER LEFT GOAL')
            self.new_st_gripper_right = ROSBridge.Dummy('GRIPPER RIGHT GOAL')

    def _exec_goal(self, goal_timeline, step, synchronous_trajectory):
        if goal_timeline[step] is not None:
            synchronous_trajectory.send_jtp_list(goal_timeline[step])

    def _exec_velocity(self, velocity_timeline, step, publisher, enable_idx):
        if velocity_timeline[step][enable_idx]:
            msg = Float64MultiArray(data=velocity_timeline[step][0:enable_idx])
            publisher.publish(msg)

    def block_velocity_timeline_for_goals(self, timeline, goal_timeline,
                                          velocity_timeline):
        for idx, jtp_list in enumerate(goal_timeline):
            if jtp_list:
                goal_duration = JTP.get_total_time_duration(jtp_list)
                timeoutSamples = int(timeline.calc_samples(
                    timeline.profile.switch_vel_to_goal_timeout))
                blockedSamples = int(timeline.calc_samples(goal_duration))
                velocity_timeline[idx-timeoutSamples:idx+blockedSamples, 7] = 0

                data_loss = velocity_timeline[idx-timeoutSamples:idx+blockedSamples] != 0
                is_data_loss = bool(data_loss.sum())

                info_msg = 'possible loss of data due to blocking arm velocity\'s for goal at sample %d' % (idx+1)
                question_msq = 'EXIT TASK?'
                if is_data_loss and PrettyOutput.attation_msg(info_msg, question_msq):
                    self._exit_task()

    def prepare_tf(self, timeline):

        timeline.TFX += timeline.profile.tf_link_ofs[0]
        timeline.TFY += timeline.profile.tf_link_ofs[1]
        timeline.TFZ += timeline.profile.tf_link_ofs[2]
        timeline.TFRoll += timeline.profile.tf_link_ofs[3]
        timeline.TFPich += timeline.profile.tf_link_ofs[4]
        timeline.TFYaw += timeline.profile.tf_link_ofs[5]



    @staticmethod
    def _exit_task():
        PrettyOutput.attation_msg('EXITING TASK')
        ROSBridge.ACTIVE_TASK = False
        exit()

    def exec_timeline(self, timeline):
        if ROSBridge.ACTIVE_TASK:
            PrettyOutput.attation_msg('ANOTHER TASK STILL RUNNING - SKIP EXECUTION OF NEW TASK')
            return

        ROSBridge.ACTIVE_TASK = True

        timeline.syncTimeline()

        self.block_velocity_timeline_for_goals(timeline=timeline,
                                               goal_timeline=timeline.ARML_GOAL,
                                               velocity_timeline=timeline.ARML_VEL)

        self.block_velocity_timeline_for_goals(timeline=timeline,
                                               goal_timeline=timeline.ARMR_GOAL,
                                               velocity_timeline=timeline.ARMR_VEL)


        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_left_param = {'translation': timeline.profile.tf_link_left_ofs[:3],
                              'rotation': tf.transformations.quaternion_from_euler(*timeline.profile.tf_link_left_ofs[3:]),
                              'child': timeline.profile.tf_link_left_name,
                              'parent': timeline.profile.tf_link_name}

        self.tf_right_param = {'translation': timeline.profile.tf_link_right_ofs[:3],
                               'rotation': tf.transformations.quaternion_from_euler(*timeline.profile.tf_link_right_ofs[3:]),
                               'child': timeline.profile.tf_link_right_name,
                               'parent': timeline.profile.tf_link_name}

        if self.exec_tf:
            self.prepare_tf(timeline=timeline)

        PrettyOutput.attation_msg('TIMELINE STARTING...')

        for step in range(timeline.get_max_length_from_timelines()):
            if self.exec_base:
                twist = Twist()
                twist.linear.x = timeline.TLX[step]
                twist.linear.y = timeline.TLY[step]
                twist.angular.z = timeline.TLTH[step]
                self.base_publisher.publish(twist)

            if self.exec_arm_left:
                self._exec_goal(goal_timeline=timeline.ARML_GOAL, step=step,
                                synchronous_trajectory=self.new_st_arm_left)
                self._exec_velocity(velocity_timeline=timeline.ARML_VEL, step=step,
                                    publisher=self.arm_left_velocity_publisher,
                                    enable_idx=7)

            if self.exec_arm_right:
                self._exec_goal(goal_timeline=timeline.ARMR_GOAL, step=step,
                                synchronous_trajectory=self.new_st_arm_right)
                self._exec_velocity(velocity_timeline=timeline.ARMR_VEL, step=step,
                                    publisher=self.arm_right_velocity_publisher,
                                    enable_idx=7)

            if self.exec_gripper_left:
                self._exec_goal(goal_timeline=timeline.GRIPPERL_GOAL, step=step,
                                synchronous_trajectory=self.new_st_gripper_left)

            if self.exec_gripper_right:
                self._exec_goal(goal_timeline=timeline.GRIPPERR_GOAL, step=step,
                                synchronous_trajectory=self.new_st_gripper_right)

            if self.exec_mimic and timeline.MIMIC[step] is not None:
                self.mimic_action_call.send_goal(timeline.MIMIC[step])


            if self.exec_led and timeline.LED[step] is not None:
                print timeline.LED[step]
                self.led_torso_call(timeline.LED[step])

            if self.exec_tf:
                self.tf_call(timeline, step)


            rospy.sleep(timeline.profile.sample_time)

        ROSBridge.ACTIVE_TASK = False

    def tf_call(self, timeline, step):
        #print timeline.TFX[step], timeline.TFY[step], timeline.TFZ[step],
        #print timeline.TFRoll[step], timeline.TFPich[step], timeline.TFYaw[step]

        self.tf_broadcaster.sendTransform((timeline.TFX[step], timeline.TFY[step], timeline.TFZ[step]),
                         tf.transformations.quaternion_from_euler(timeline.TFRoll[step], timeline.TFPich[step], timeline.TFYaw[step]),
                         rospy.Time.now(), timeline.profile.tf_link_name, timeline.profile.tf_source_name)

        self.tf_broadcaster.sendTransform(time=rospy.Time.now(), **self.tf_left_param)
        self.tf_broadcaster.sendTransform(time=rospy.Time.now(), **self.tf_right_param)


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
        self.is_ros_gripper_left = False
        self.is_ros_gripper_right = False
        self.is_ros_base = False
        self.is_ros_mimic = False
        self.is_ros_led = False

        self.is_ros_tf = False

        self.is_lab = False

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

        rospy.Service('/scenario/enable_gripper_left', Trigger, self._enable_gripper_left)
        rospy.Service('/scenario/disable_gripper_left', Trigger, self._disable_gripper_left)

        rospy.Service('/scenario/enable_gripper_right', Trigger, self._enable_gripper_right)
        rospy.Service('/scenario/disable_gripper_right', Trigger, self._disable_gripper_right)

        rospy.Service('/scenario/enable_mimic', Trigger, self._enable_mimic)
        rospy.Service('/scenario/disable_mimic', Trigger, self._disable_mimic)

        rospy.Service('/scenario/enable_led', Trigger, self._enable_led)
        rospy.Service('/scenario/disable_led', Trigger, self._disable_led)

        rospy.Service('/scenario/enable_tf', Trigger, self._enable_tf)
        rospy.Service('/scenario/disable_tf', Trigger, self._disable_tf)

    def init_startup_args(self):
        self.is_fakerun = '-fakerun' in sys.argv
        self.is_service_mode = '-servicemode' in sys.argv

        self.is_lab = '-lab' in sys.argv

        self.is_plot = '-plot' in sys.argv
        if self.is_plot:
            idx = sys.argv.index('-plot') + 1
            self.plot_map = 'map' in sys.argv[idx:idx+2]
            self.plot_profile = 'profile' in sys.argv[idx:idx+2]

        self.is_ros = '-ros' in sys.argv
        if self.is_ros:
            n_ros_params = 8
            idx = sys.argv.index('-ros') + 1
            self.is_ros_arm_left = 'arm_left' in sys.argv[idx:idx+n_ros_params]
            self.is_ros_arm_right = 'arm_right' in sys.argv[idx:idx+n_ros_params]
            self.is_ros_gripper_left = 'gripper_left' in sys.argv[idx:idx+n_ros_params]
            self.is_ros_gripper_right = 'gripper_right' in sys.argv[idx:idx+n_ros_params]
            self.is_ros_base = 'base' in sys.argv[idx:idx+n_ros_params]
            self.is_ros_mimic = 'mimic' in sys.argv[idx:idx+n_ros_params]
            self.is_ros_led = 'led' in sys.argv[idx:idx+n_ros_params]
            self.is_ros_tf = 'tf' in sys.argv[idx:idx+n_ros_params]


    def print_startup_args(self, *_):
        print
        print
        print 'FAKERUN:          ', self.is_fakerun
        print
        print 'SERVICE MODE:     ', self.is_service_mode
        print
        print 'ROS ENABLED:      ', self.is_ros
        print '    ARM LEFT:     ', self.is_ros_arm_left
        print '    ARM RIGHT:    ', self.is_ros_arm_right
        print '    GRIPPER LEFT: ', self.is_ros_gripper_left
        print '    GRIPPER RIGHT:', self.is_ros_gripper_right
        print '    BASE:         ', self.is_ros_base
        print
        print '    MIMIC:        ', self.is_ros_mimic
        print '    LED:          ', self.is_ros_led
        print
        print '    TF :          ', self.is_ros_tf
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
        if not self.is_service_mode:
            return

        if not isinstance(func_list, list):
            func_list = [func_list]

        if self.is_lab:
            func_list = [getattr(bound_timline_object, 'lab_' + fnc.__name__, fnc) for fnc in func_list]

        for idx, fnc in enumerate(func_list):
            pass

        rospy.Service(service_name, Trigger, self.callback_creator(service_name, func_list, bound_timline_object))

    def do_listen(self):
        if not self.is_service_mode:
            PrettyOutput.attation_msg('LISTENING ONLY IN SERVICEMODE')
            return
        rospy.spin()

    def _inplace_restart(self, *args):
        for _ in range(50): print
        PrettyOutput.attation_msg('RESTARTING APPLICATION')
        os.execv(sys.argv[0], sys.argv)

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

        if not (self.is_ros_base or self.is_ros_arm_left or self.is_ros_arm_right or self.is_ros_mimic
                or self.is_ros_led or self.is_ros_gripper_left or self.is_ros_gripper_right or self.is_ros_tf):
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

    def _enable_gripper_left(self, *args):
        self._enable_argv('gripper_left', self.is_ros_gripper_left)

    def _disable_gripper_left(self, *args):
        self._disable_argv('gripper_left', self.is_ros_gripper_left)

    def _enable_gripper_right(self, *args):
        self._enable_argv('gripper_right', self.is_ros_gripper_right)

    def _disable_gripper_right(self, *args):
        self._disable_argv('gripper_right', self.is_ros_gripper_right)

    def _enable_mimic(self, *args):
        self._enable_argv('mimic', self.is_ros_mimic)

    def _disable_mimic(self, *args):
        self._disable_argv('mimic', self.is_ros_mimic)

    def _enable_led(self, *args):
        self._enable_argv('led', self.is_ros_led)

    def _disable_led(self, *args):
        self._disable_argv('led', self.is_ros_led)

    def _enable_tf(self, *args):
        self._enable_argv('tf', self.is_ros_tf)

    def _disable_tf(self, *args):
        self._disable_argv('tf', self.is_ros_tf)

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
                               exec_arm_right=self.is_ros_arm_right,
                               exec_mimic=self.is_ros_mimic,
                               exec_led=self.is_ros_led,
                               exec_gripper_left=self.is_ros_gripper_left,
                               exec_gripper_right=self.is_ros_gripper_right,
                               exec_tf=self.is_ros_tf)

            bridge.exec_timeline(timeline)


    def startup(self, timeline):

        PrettyOutput.attation_msg('APPLICATION STARTED')

        if not self.is_service_mode:
            self.execute_timeline(timeline)
        else:
            self.do_listen()


class Profile(object):
    def __init__(self, rate, max_linear_velocity, max_angular_velocity, max_linear_acceleration, max_angular_acceleration, switch_vel_to_goal_timeout):
        self.rate = float(rate)  # [Hz]
        self.sample_time = 1.0 / rate  # [s]

        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        self.max_linear_acceleration = max_linear_acceleration
        self.max_angular_acceleration = max_angular_acceleration
        self.switch_vel_to_goal_timeout = switch_vel_to_goal_timeout

        self.tf_source_name = 'base_link'
        self.tf_link_name = 'ball_link'
        self.tf_link_ofs = [0.6, 0, 0.7, 0, 0, 0]
        self.tf_link_left_name = self.tf_link_name + '_left'
        self.tf_link_left_ofs = [0, 0.25, 0, np.pi/2, 0, 0]
        self.tf_link_right_name = self.tf_link_name + '_right'
        self.tf_link_right_ofs = [0, -0.25, 0, -np.pi/2, np.pi, 0]


class Timeline(object):
    def __init__(self, profile):
        self.profile = profile
        self.TLX = None
        self.TLY = None
        self.TLTH = None
        self.ARML_GOAL = None
        self.ARMR_GOAL = None
        self.GRIPPERL_GOAL = None
        self.GRIPPERR_GOAL = None
        self.ARML_VEL = None
        self.ARMR_VEL = None
        self.MIMIC = None
        self.LED = None
        self.SECTIONS = None
        self.TFX = None
        self.TFY = None
        self.TFZ = None
        self.TFRoll = None
        self.TFPich = None
        self.TFYaw = None

        self.clear_data()

    def clear_data(self):
        self.TLX = np.array([], np.float64)
        self.TLY = np.array([], np.float64)
        self.TLTH = np.array([], np.float64)
        self.ARML_GOAL = list()
        self.ARMR_GOAL = list()
        self.GRIPPERL_GOAL = list()
        self.GRIPPERR_GOAL = list()
        self.ARML_VEL = np.ndarray((0, 8), np.float64)
        self.ARMR_VEL = np.ndarray((0, 8), np.float64)
        self.MIMIC = list()
        self.LED = list()
        self.SECTIONS = list()

        self.TFX = np.array([], np.float64)
        self.TFY = np.array([], np.float64)
        self.TFZ = np.array([], np.float64)
        self.TFRoll = np.array([], np.float64)
        self.TFPich = np.array([], np.float64)
        self.TFYaw = np.array([], np.float64)

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

    def appendGripperLeft(self, gripper_left_data):
        if not gripper_left_data:
            return
        self.GRIPPERL_GOAL.append(gripper_left_data)

    def appendGripperRight(self, gripper_right_data):
        if not gripper_right_data:
            return
        self.GRIPPERR_GOAL.append(gripper_right_data)

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

    def appendMimic(self, name='default', speed=0.0, repeat=0):
        if not isLive:
            return

        mag = SetMimicGoal()
        mag.mimic = name
        mag.speed = speed
        mag.repeat = repeat
        self.MIMIC.append(mag)

    def appendLed(self, r=0, g=1, b=0.7, a=0.4, frequency=0.25, mode=3):
        luminosity_scale = 1

        lmr = SetLightModeRequest()
        lmr.mode.mode = mode
        lmr.mode.color.r = r * luminosity_scale
        lmr.mode.color.g = g * luminosity_scale
        lmr.mode.color.b = b * luminosity_scale
        lmr.mode.color.a = a
        lmr.mode.frequency = frequency
        self.LED.append(lmr)

    def appendTFX(self, data):
        self.TFX = np.append(self.TFX, data)

    def appendTFY(self, data):
        self.TFY = np.append(self.TFY, data)

    def appendTFZ(self, data):
        self.TFZ = np.append(self.TFZ, data)

    def appendTFRoll(self, data):
        self.TFRoll = np.append(self.TFRoll, data)

    def appendTFPich(self, data):
        self.TFPich = np.append(self.TFPich, data)

    def appendTFYaw(self, data):
        self.TFYaw = np.append(self.TFYaw, data)

    def syncTF(self, onlyTF=False, syncToMax=True):
        tf_list = [self.TFX, self.TFY, self.TFZ, self.TFRoll, self.TFPich, self.TFYaw]
        operation = max if syncToMax else min
        tf_lens = map(len, tf_list)
        target_size = operation(tf_lens) if onlyTF else self.get_max_length_from_timelines()
        size_remaining_list = [target_size - tf_len for tf_len in tf_lens]
        last_value_list = [tf[-1] if len(tf) else 0 for tf in tf_list]
        synced = [np.append(tf, np.array([last_value]*remaining, np.float64))
                  for tf, last_value, remaining in zip(tf_list, last_value_list, size_remaining_list)]
        self.TFX, self.TFY, self.TFZ, self.TFRoll, self.TFPich, self.TFYaw = synced



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
        self.syncTimelineMimic()
        self.syncTimelineLed()
        self.syncTimelineGripper()
        self.syncTF()

    def syncTimelineBase(self):
        self.TLX, self.TLY, self.TLTH = self._evenMaxSamples(np.zeros, [np.float64], 0, self.TLX, self.TLY, self.TLTH)

    def syncTimelineArmVelocity(self):
        self.ARML_VEL, self.ARMR_VEL = self._evenMaxSamples(np.zeros, [np.float64], 8, self.ARML_VEL, self.ARMR_VEL)

    def syncTimelineArmGoal(self):
        self.ARML_GOAL, self.ARMR_GOAL = self._evenMaxSamples(self._generatePythonList, [None], 0, self.ARML_GOAL, self.ARMR_GOAL)

    def syncTimelineGripper(self):
        self.GRIPPERL_GOAL, self.GRIPPERR_GOAL= self._evenMaxSamples(self._generatePythonList, [None], 0, self.GRIPPERL_GOAL, self.GRIPPERR_GOAL)

    def syncTimelineMimic(self):
        self.MIMIC = self._evenMaxSamples(self._generatePythonList, [None], 0, self.MIMIC)

    def syncTimelineLed(self):
        self.LED = self._evenMaxSamples(self._generatePythonList, [None], 0, self.LED)




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

        return args if len(args) > 1 else args[0]

    def get_max_length_from_timelines(self):
        timelines = [self.TLX, self.TLY, self.TLTH, self.ARML_GOAL, self.ARMR_GOAL, self.ARML_VEL, self.ARMR_VEL]
        timelines.extend([self.TFX, self.TFY, self.TFZ, self.TFRoll, self.TFPich, self.TFYaw])
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

        nSamples = duration * self.profile.rate

        xvals, yvals = self.bezier_curve(points, nTimes=nSamples)

        xdiff = np.diff(xvals)
        ydiff = np.diff(yvals)

        rel_distance = np.sqrt(xdiff**2 + ydiff**2)
        abs_phi = np.arctan2(xdiff, ydiff)

        rel_phi = np.diff(abs_phi)
        start_phi = np.array([0], np.float)
        rel_phi = np.append(start_phi, rel_phi)
        corridx = [(idx+1, 1 if phi < 0 else -1) for idx, phi in enumerate(rel_phi) if abs(phi) > 6]
        for k, v in corridx:
            abs_phi[k:] += np.pi*2*v
        rel_phi = np.diff(abs_phi)
        start_phi = np.array([0], np.float)
        rel_phi = np.append(start_phi, rel_phi)
        rel_phi = np.append(rel_phi, start_phi)

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

        positions = [np.round(p, 2) for p in positions]

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
    def load_trajectory_goal(filename):

        with open(filename, 'r') as f:
            trajectory_goal_data = yaml.safe_load(f)

        jtp_list = [list(), list()]
        for left_data in trajectory_goal_data['left']:
            jtp_list[0].append(JTP(1, *left_data))

        for right_data in trajectory_goal_data['right']:
            jtp_list[1].append(JTP(1, *right_data))

        return jtp_list

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

class NewSynchronousTrajectory(object):
    def __init__(self, logic_name, joint_trajectory_topic, joint_names, do_init=False, timeout=5):
        self.do_init = do_init
        self.joint_names = joint_names
        self.logic_name = logic_name

        if do_init:
            self.action_client = actionlib.SimpleActionClient(joint_trajectory_topic, FollowJointTrajectoryAction)
            if not self.action_client.wait_for_server(timeout=rospy.Duration(timeout)):
                PrettyOutput.init_exit_msg(logic_name)


    def send_jtp_list(self, jtp_list):
        if not self.do_init:
            return

        print 'Sending following JTP-Points to %s:' % self.logic_name
        for jtp in jtp_list:
            print jtp

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        goal.trajectory.points = JTP.get_point_list(jtp_list)
        self.action_client.send_goal(goal=goal)
        return JTP.get_total_time_duration(jtp_list)

    @staticmethod
    def _resolve_goal_stats_codes():
        CODENAMES = [code for code in dir(GoalStatus) if not code.startswith('_') and code[0] in string.ascii_uppercase]
        CODENUMS = map(getattr, [GoalStatus] * len(CODENAMES), CODENAMES)
        goal_status_dict = dict()
        map(lambda num, name: goal_status_dict.update({num: name}), CODENUMS, CODENAMES)
        return goal_status_dict


class GripperMovement(object):

    gripper_pose_close = {'p1': -0.85, 'p2':  0.1}
    gripper_pose_home =  {'p1': -0.6,  'p2': -0.4}
    gripper_pose_open =  {'p1':  0.85, 'p2': -1.4}

    gripper_pose_rose_open =  {'p1':  0.85, 'p2': -1.4}


    def __init__(self, profile):
        self.profile = profile


class ArmMovement(object):
    pose_home = {'p1': 0, 'p2': 0, 'p3': 0, 'p4': 0, 'p5': 0, 'p6': 0, 'p7': 0,
                 'v1': 0, 'v2': 0, 'v3': 0, 'v4': 0, 'v5': 0, 'v6': 0, 'v7': 0}

    pose_boring_walk_back = {'p1': np.radians(-55), 'p2': np.radians(-95),
                             'p3': np.radians(60), 'p4': np.radians(95),
                             'p5': np.radians(60), 'p6': np.radians(40),
                             'p7': -0.71,
                             'v1': 0, 'v2': 0, 'v3': 0, 'v4': 0, 'v5': 0, 'v6': 0, 'v7': 0}


    pose_boring_walk_front_back_c1 = {'p1': np.radians(-80), 'p2': np.radians(-74),
                                      'p3': np.radians(100), 'p4': np.radians(94),
                                      'p5': np.radians(60), 'p6': np.radians(40),
                                      'p7': -0.95}


    pose_boring_walk_front = {'p1': np.radians(-110), 'p2': np.radians(-75),
                              'p3': np.radians(147), 'p4': np.radians(90),
                              'p5': np.radians(60), 'p6': np.radians(40),
                              'p7': -0.85,
                              'v1': 0, 'v2': 0, 'v3': 0, 'v4': 0, 'v5': 0, 'v6': 0, 'v7': 0}

    pose_shock_front_left = {'p1': -1.97, 'p2': -1.30,
                             'p3': 2.46, 'p4': 1.80,
                             'p5': 0.99, 'p6': 0.81,
                             'p7': -1.42}

    pose_shock_front_right = {'p1': 1.87, 'p2': 1.35,
                              'p3': -2.16, 'p4': -1.17,
                              'p5': -1.47, 'p6': -1.24,
                              'p7': 1.37}

    pose_run_arms = {'p1': np.radians(-60), 'p2': np.radians(-75),
                     'p3': np.radians(80), 'p4': np.radians(105),
                     'p5': np.radians(40), 'p6': np.radians(20),
                     'p7': -0.83}


    pose_cheer_arms_up = {'p1': np.radians(-172), 'p2': np.radians(-83),
                          'p3': np.radians(105), 'p4': np.radians(100),
                          'p5': np.radians(0), 'p6': np.radians(20)}

    pose_waiting_arms_side = {'p1': np.radians(-45), 'p2': np.radians(-60),
                              'p3': np.radians(105), 'p4': np.radians(90),
                              'p5': np.radians(10), 'p6': np.radians(50)}

    # ROSE GREIFEN

    pose_right_folded_back = {'p1': np.radians(55), 'p2': np.radians(90),
                              'p3': np.radians(0), 'p4': np.radians(63),
                              'p5': np.radians(83), 'p6': np.radians(40),
                              'p7': -1.6}

    pose_folded_grip_right_c1 = {'p1': np.radians(57), 'p2': np.radians(96),
                                 'p3': np.radians(-17), 'p4': np.radians(0),
                                 'p5': np.radians(14), 'p6': np.radians(-90)}

    pose_folded_grip_right_c2 = {'p1': 1.59, 'p2': 1.12,
                                 'p3': -1.70, 'p4': -1.5,
                                 'p5': 0, 'p6': -1.08}

    pose_folded_grip_right_c3 = {'p1': 2.34, 'p2': 1.20,
                                 'p3': -2.0, 'p4': -1.7,
                                 'p5': 0.38, 'p6': -0.75}

    pose_folded_grip_right_c4 = {'p1': 3.14, 'p2': 1.66,
                                 'p3': -1.93, 'p4': -1.73,
                                 'p5': 0.31, 'p6': 1.37,
                                 'p7': 0.35}

    pose_grip_rose_right = {'p1': 3.54, 'p2': 1.83,
                                 'p3': -1.93, 'p4': -0.45,
                                 'p5': 0.31, 'p6': 0.77,
                                 'p7': 0.19}

    pose_folded_grip_right_c6 = {'p1': 3.79, 'p2': 1.83,
                                 'p3': -1.93, 'p4': -0.20,
                                 'p5': 0.31, 'p6': 0.19,
                                 'p7': 0.41}

    pose_carry_rose_front_right = {'p1': 2.34, 'p2': 1.15,
                                   'p3': -2.69, 'p4': -1.96,
                                   'p5': 0.38, 'p6': 0.16,
                                   'p7': 0.40}

    pose_cheer_drum = {'p1': -2.8, 'p2': -1.39,
                       'p3': 1.97, 'p4': 1.37,
                       'p5': 0, 'p6': -0.18}

    pose_cheer_turn = {'p1': -4.13, 'p2': -1.20,
                       'p3': 2.4, 'p4': 1.17,
                       'p5': 0, 'p6': 0.35}

    pose_present_give_rose_right = {'p1': 2.91, 'p2': 1.50,
                                    'p3': -2.39, 'p4': -1.28,
                                    'p5': 0.62, 'p6': 0.64,
                                    'p7': 0.34,
                                    'v1': 0, 'v2': 0, 'v3': 0, 'v4': 0, 'v5': 0, 'v6': 0, 'v7': 0}

    def __init__(self, profile):
        self.profile = profile

    def inject_zero_velocity(self, pose):
        pargs = dict(pose)
        pargs.update({'v1': 0, 'v2': 0, 'v3': 0, 'v4': 0, 'v5': 0, 'v6': 0, 'v7': 0})
        return pargs

    def movePose(self, pose, duration=4):
        jtp_list_left = list()

        jtp_list_left.append(JTP(rel_time=duration, **pose))
        jtp_list_right = JTP.get_mirrored_jtp_list(jtp_list_left)

        return jtp_list_left, jtp_list_right


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

        pose_list_left = [ArmMovement.pose_boring_walk_front_back_c1, ArmMovement.pose_boring_walk_front,
                          ArmMovement.pose_boring_walk_front_back_c1, ArmMovement.pose_boring_walk_back]

        pose_list_right = [ArmMovement.pose_boring_walk_front_back_c1, ArmMovement.pose_boring_walk_back,
                           ArmMovement.pose_boring_walk_front_back_c1, ArmMovement.pose_boring_walk_front]

        jtp_flow_data = JTP.extend_base_list(None, *self.movePoseSplit(pose_list_left=pose_list_left,
                                                                       pose_list_right=pose_list_right,
                                                                       duration=dotime_step, times=times))

        JTP.extend_base_list(jtp_flow_data, *self.movePose(ArmMovement.pose_boring_walk_front_back_c1, dotime_step))

        return jtp_flow_data


class Bricks(object):
    def __init__(self, profile):
        self.profile = profile

    def calc_samples(self, duration):
        return int(duration / self.profile.sample_time)

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
        tlx_, tlth = self.circular_path(radius=1, phi=phi, duration=duration, acc_percentage=0.2, dec_percentage=0.2)

        absphi = integrate.cumtrapz(tlth*self.profile.sample_time)
        absphi  = absphi[:-1]

        tlx = velocity_start_x * np.cos(absphi) - velocity_start_y * np.sin(absphi)
        tly = velocity_start_x * -np.sin(absphi) + velocity_start_y * np.cos(absphi)

        if velocity_start_y < 0:
            tlx *= -1

        return tlx, tly, tlth[1:-1]


class BaseScene(Timeline, Bricks, Bezier, ArmMovement, GripperMovement):
    def __init__(self, profile):
            super(BaseScene, self).__init__(profile)

class DummyScene(BaseScene):
    def __init__(self, profile):
            super(DummyScene, self).__init__(profile)


if __name__ == '__main__':
    PrettyOutput.attation_msg('DIRECT EXECUTION OF SCRIPT FORBIDDEN')