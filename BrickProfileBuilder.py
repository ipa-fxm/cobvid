#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import numpy as np
import time
import sys

class Profile(object):
    def __init__(self, rate, max_linear_velocity, max_angular_velocity):
        self.rate = float(rate)  # [Hz]
        self.sample_time = 1.0 / rate  # [s]

        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity



class Bricks(object):
    def __init__(self, profile):
        self.profile = profile

    def calc_samples(self, duration):
        return duration / self.profile.sample_time

    def evenMaxSamples(self, *args):
        args = list(args)
        max_samples = max(map(len, args))
        for idx, sample_line in enumerate(args):
            remaining_samples = max_samples - len(sample_line)
            fill_data = np.zeros((remaining_samples), np.float)
            args[idx] = np.append(sample_line, fill_data)
        return args


    def sin(self, start, stop, duration):
        return np.sin(np.linspace(start, stop, self.calc_samples(duration)))

    def sin_t(self, A, f, phi, T):
        #return np.sin(np.linspace(0, np.pi*2 * f*T, self.calc_samples(T)))
        return A * self.sin(phi, f*np.pi*2*T+phi, T)

    def cos(self, start, stop, duration):
        return np.cos(np.linspace(start, stop, self.calc_samples(duration)))

    def lin(self, duration, velocity=1):
        return np.linspace(velocity, velocity, self.calc_samples(duration))

    def lin_dist(self, distance, duration):
        speed = float(distance) / float(duration)
        print speed
        return self.lin(duration) * speed

    def acc(self, velocity_start, velocity_end, duration):
        if velocity_start < velocity_end:
            sin_intv = self.sin(-np.pi/2, np.pi/2, duration)
            velocity_low, velocity_hi = velocity_start, velocity_end
        else:
            sin_intv = self.sin(np.pi/2, np.pi*3/2, duration)
            velocity_low, velocity_hi = velocity_end, velocity_start
        return (sin_intv + 1) / 2 * (velocity_hi - velocity_low) + velocity_low

    def circular_path_parameter(self, duration, radius, phi):
        dphi = phi / self.calc_samples(duration)
        theta = dphi / self.profile.sample_time
        velocity = radius * theta
        return velocity, theta




class GeneralMovement(object):
    def __init__(self, profile):
        self.profile = profile

    def move_linear(self, speed, duration):
        return

class BoringMovement(object):
    def __init__(self, profile):
        self.profile = profile


class ROSBridge(object):
    class Dummy(object):
        pass

    def __init__(self, profile, fakerun=False):
        self.profile = profile

        if not fakerun:
            rospy.init_node('VID_TEST')
            self.pub = rospy.Publisher('/base_controller/command_direct', Twist)
        else:
            self.pub = ROSBridge.Dummy()
            self.pub.publish = self.print_fakerun

    def print_fakerun(self, msg):
        print msg
        print '-'*50

    def exec_timeline(self, timeline):

        for step in range(max([len(timeline['x']), len(timeline['y']), len(timeline['th'])])):
            twist = Twist()
            if step < len(timeline['x']):
                twist.linear.x = timeline['x'][step]
            if step < len(timeline['y']):
                twist.linear.y = timeline['y'][step]
            if step < len(timeline['th']):
                twist.angular.z = timeline['th'][step]

            self.pub.publish(twist)
            time.sleep(self.profile.sample_time)


if __name__ == '__main__':

    is_fakerun = True if '-fakerun' in sys.argv else False
    is_plot = True if '-plot' in sys.argv else False
    is_ros = True if '-ros' in sys.argv else False


    TLX = np.array([], np.float)
    TLY = np.array([], np.float)
    TLTH = np.array([], np.float)

    profile = Profile(rate=100, max_linear_velocity=0.2, max_angular_velocity=0.4)
    bricks = Bricks(profile)

    boring = BoringMovement(profile)

    # wait 0.5 sec
    TLX = np.append(TLX, bricks.lin(duration=0.5, velocity=0))

    # sync lines
    TLX, TLY, TLTH = bricks.evenMaxSamples(TLX, TLY, TLTH)


    # KREISBAHN
    ############

    velocity, theta = bricks.circular_path_parameter(duration=10, radius=1, phi=np.pi/2)

    # accelerate
    TLX = np.append(TLX, bricks.acc(velocity_start=0, velocity_end=velocity, duration=1.5))

    # sync lines
    TLX, TLY, TLTH = bricks.evenMaxSamples(TLX, TLY, TLTH)

    # circular movement
    TLX = np.append(TLX, bricks.lin(duration=10, velocity=velocity))
    TLTH = np.append(TLTH, bricks.lin(duration=10, velocity=velocity))

    # decelerate
    TLX = np.append(TLX, bricks.acc(velocity_start=velocity, velocity_end=0, duration=1.5))

    # sync lines
    TLX, TLY, TLTH = bricks.evenMaxSamples(TLX, TLY, TLTH)



    #TLTH = np.append(TLTH, bricks.lin(duration=2, velocity=0))
    #TLTH = np.append(TLTH, bricks.lin(3)*-0.15)  # TODO: kreisbahn... v, r, th


    TIMELINE = dict()
    TIMELINE['x'] = TLX
    TIMELINE['y'] = TLY
    TIMELINE['th'] = TLTH

    if is_ros:
        bridge = ROSBridge(profile, fakerun=is_fakerun)
        bridge.exec_timeline(TIMELINE)

    if is_plot:
        import matplotlib.pyplot as plt

        fig, (ax) = plt.subplots(nrows=1, ncols=1)
        TLX, TLY, TLTH = bricks.evenMaxSamples(TLX, TLY, TLTH)
        max_samples = max([len(TLX), len(TLY), len(TLTH)])
        xdata = np.linspace(0, profile.sample_time * max_samples, max_samples)

        ax.plot(xdata, TLX)
        ax.plot(xdata, TLY)
        ax.plot(xdata, TLTH)


        ax.legend(['X', 'Y', 'Theta'])

        plt.tight_layout()
        plt.show()

