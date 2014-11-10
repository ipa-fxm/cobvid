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
        twist = Twist()
        for d in timeline:
            #twist.linear.x = d
            twist.angular.z = d
            self.pub.publish(twist)
            time.sleep(self.profile.sample_time)



if __name__ == '__main__':

    TIMELINE = np.array([], np.float)

    profile = Profile(rate=100, max_linear_velocity=0.2, max_angular_velocity=0.4)
    bricks = Bricks(profile)

    boring = BoringMovement(profile)

    #TIMELINE = bricks.lin_dist(0.25, 4)
    #TIMELINE = np.append(TIMELINE, bricks.lin_dist(0.25, 2))

    #print TIMELINE
    # A f phi T
    TIMELINE = bricks.sin_t(0.2, 1, np.pi/2, 5) 
    
    
    

    bridge = ROSBridge(profile)
    bridge.exec_timeline(TIMELINE)

    fig, (ax) = plt.subplots(nrows=1, ncols=1)
    ax.plot(TIMELINE)

    plt.tight_layout()
    #plt.show()

