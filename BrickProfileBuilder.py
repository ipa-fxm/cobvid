#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import numpy as np
import time

import matplotlib.pyplot as plt

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

    def sin(self, start, stop, duration):
        return np.sin(np.linspace(start, stop, self.calc_samples(duration)))

    def sin_t(self, A, f, phi, T):
        #return np.sin(np.linspace(0, np.pi*2 * f*T, self.calc_samples(T)))
        return A * self.sin(phi, f*np.pi*2*T+phi, T)
    
    
    def cos(self, start, stop, duration):
        return np.cos(np.linspace(start, stop, self.calc_samples(duration)))

    def lin(self, duration):
        return np.linspace(1, 1, self.calc_samples(duration))

    def lin_dist(self, distance, duration):
        speed = float(distance) / float(duration)
        print speed
        return self.lin(duration) * speed

    def acc(self, vel_start, vel_end, duration):
        if vel_start < vel_end:
            sin_intv = self.sin(-np.pi/2, np.pi/2, duration)
            vel_low, vel_hi = vel_start, vel_end
        else:
            sin_intv = self.sin(np.pi/2, np.pi*3/2, duration)
            vel_low, vel_hi = vel_end, vel_start
        return (sin_intv + 1) / 2 * (vel_hi - vel_low) + vel_low

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

    TIMELINE = dict()
    TIMELINE['x'] = np.array([], np.float)
    TIMELINE['y'] = np.array([], np.float)
    TIMELINE['th'] = np.array([], np.float)

    profile = Profile(rate=100, max_linear_velocity=0.2, max_angular_velocity=0.4)
    bricks = Bricks(profile)

    boring = BoringMovement(profile)

    TIMELINE['x'] = np.append(TIMELINE['x'], bricks.lin(0.5)*0)
    #TIMELINE['x'] = np.append(TIMELINE['x'], bricks.lin(2)*0.2)

    #TIMELINE['x'] = np.append(TIMELINE['x'], bricks.lin_dist(0, 1))

    #acc prof (speed, dur)
    TIMELINE['x'] = np.append(TIMELINE['x'], bricks.acc(0, 0.2, 1.5))
    TIMELINE['x'] = np.append(TIMELINE['x'], bricks.lin(3)*0.2)
    TIMELINE['x'] = np.append(TIMELINE['x'], bricks.acc(0.2, 0, 1.5))
    #TIMELINE['x'] = np.append(TIMELINE['x'], bricks.acc(0.4, 0, 1.5))


    # A f phi T
    TIMELINE['th'] = np.append(TIMELINE['th'], bricks.lin(2)*0)
    TIMELINE['th'] = np.append(TIMELINE['th'], bricks.lin(3)*-0.15)  # TODO: kreisbahn... v, r, th
    #TIMELINE['th'] = np.append(TIMELINE['th'], bricks.lin_dist(0, 1))
    #TIMELINE['th'] = np.append(TIMELINE['th'], bricks.sin_t(0.2, 1.0/5, 0, 5))


    bridge = ROSBridge(profile, fakerun=False)
    bridge.exec_timeline(TIMELINE)


    fig, (ax) = plt.subplots(nrows=1, ncols=1)
    ax.plot(TIMELINE['x'])

    plt.tight_layout()
    #plt.show()

