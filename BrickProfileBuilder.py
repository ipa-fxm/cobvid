#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import numpy as np
import time
import sys
from scipy import integrate

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

    def circular_path(self, radius, phi, duration):
        th_max = phi / (duration*0.9)

        anz_samples1 = self.calc_samples(duration*0.1)
        tdata1 = np.linspace(0, profile.sample_time * anz_samples1, anz_samples1)

        th_t = th_max/2.0 * (-np.cos(10*np.pi/duration* tdata1) + 1)
        th_t_reverse = np.copy(th_t[::-1])
        th_t = np.append(th_t, self.lin(velocity=th_max, duration=duration*0.8))
        th_t = np.append(th_t, th_t_reverse)

        v_t = th_t * radius
        return v_t, th_t


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

    # vor links
    tlx, tlth = bricks.circular_path(radius=0.5, phi=np.pi/2, duration=6)
    TLX = np.append(TLX, tlx)
    TLTH = np.append(TLTH, tlth)

    '''
    # rück links
    tlx, tlth = bricks.circular_path(radius=-0.5, phi=np.pi/2, duration=6)
    TLX = np.append(TLX, tlx)
    TLTH = np.append(TLTH, tlth)
    '''

    '''    # vor rechts
    tlx, tlth = bricks.circular_path(radius=0.5, phi=-np.pi/2, duration=6)
    TLX = np.append(TLX, tlx)
    TLTH = np.append(TLTH, tlth)

    # rück rechts
    tlx, tlth = bricks.circular_path(radius=-0.5, phi=-np.pi/2, duration=6)
    TLX = np.append(TLX, tlx)
    TLTH = np.append(TLTH, tlth)
    '''


    # sync lines
    TLX, TLY, TLTH = bricks.evenMaxSamples(TLX, TLY, TLTH)



    TIMELINE = dict()
    TIMELINE['x'] = TLX
    TIMELINE['y'] = TLY
    TIMELINE['th'] = TLTH

    if is_ros:
        bridge = ROSBridge(profile, fakerun=is_fakerun)
        bridge.exec_timeline(TIMELINE)

    if is_plot:
        import matplotlib.pyplot as plt
        import scipy

        fig, (ax1, ax2, ax3) = plt.subplots(nrows=3, ncols=1)
        legend = ['x', 'y', '$\\theta$']

        TLX, TLY, TLTH = bricks.evenMaxSamples(TLX, TLY, TLTH)
        max_samples = max([len(TLX), len(TLY), len(TLTH)])
        xdata = np.linspace(0, profile.sample_time * max_samples, max_samples)

        TLXD = np.array([0], np.float)
        TLYD = np.array([0], np.float)
        TLTHD = np.array([0], np.float)

        TLXD = np.append(TLXD, integrate.cumtrapz(TLX, xdata))
        TLYD = np.append(TLYD, integrate.cumtrapz(TLY, xdata))
        TLTHD = np.append(TLTHD, integrate.cumtrapz(TLTH, xdata))

        ax1.plot(xdata, TLXD)
        ax1.plot(xdata, TLYD)
        ax1.plot(xdata, TLTHD)
        ax1.legend(legend)
        ax1.set_title('Distance')

        ##################################################


        ax2.plot(xdata, TLX)
        ax2.plot(xdata, TLY)
        ax2.plot(xdata, TLTH)
        ax2.legend(legend)
        ax2.set_title('Velocity')

        ##################################################


        TLXA = np.array([0], np.float)
        TLYA = np.array([0], np.float)
        TLTHA = np.array([0], np.float)

        TLXA = np.append(TLXA, scipy.diff(TLX))
        TLYA = np.append(TLYA, scipy.diff(TLY))
        TLTHA = np.append(TLTHA, scipy.diff(TLTH))

        ax3.plot(xdata, TLXA)
        ax3.plot(xdata, TLYA)
        ax3.plot(xdata, TLTHA)
        ax3.legend(legend)
        ax3.set_title('Acceleration')

        ##################################################
        # MAP
        ##################################################

        '''
        fig, (ax1) = plt.subplots(nrows=1, ncols=1)
        TLTHDS = np.sin(TLTHD)
        TLTHDC = np.cos(TLTHD)
        TLPX = TLXD * TLTHDS
        TLPY = TLXD * TLTHDC
        ax1.plot(TLPX, TLPY)
        ax1.set_xlabel('x [m]')
        ax1.set_ylabel('y [m]')
        ax1.legend(['Path'])

        plt.axis('equal')
        '''

        ##################################################
        # TESTING AREA
        ##################################################


        fig, (ax2, ax1) = plt.subplots(nrows=1, ncols=2)

        phi = np.pi
        R = 1.5
        T = 6

        # NEW TEST

        anz_samples = bricks.calc_samples(T)
        tdata =  np.linspace(0, profile.sample_time * anz_samples, anz_samples)

        v_t, th_t = bricks.circular_path(radius=R, phi=np.pi / 2, duration=T)

        absphi = np.array([0], np.float)
        absphi = np.append(absphi, integrate.cumtrapz(th_t, tdata))

        ax2.plot(tdata, th_t)
        ax2.plot(tdata, v_t)
        ax2.set_title('Velocity')
        ax2.legend(['$\\theta(t)$ [$rad \\cdot s^{-1}$]', '$v(t)$ [$m \\cdot s^{-1}$]'])

        ax2.set_xlabel('t [s]')
        ax2.set_ylabel('v')

        # Plot

        xdata = R * np.cos(absphi) - R
        ydata = R * np.sin(absphi)


        ax1.plot(xdata, ydata, '-', color=(0, 0, 0), alpha=0.5)

        start, end = 0.0, 0.1
        ax1.plot(xdata[np.floor(anz_samples*start):np.floor(anz_samples*end)], ydata[np.floor(anz_samples*start):np.floor(anz_samples*end)], 'gx-')

        start, end = 0.1, 0.9
        ax1.plot(xdata[np.floor(anz_samples*start):np.floor(anz_samples*end)], ydata[np.floor(anz_samples*start):np.floor(anz_samples*end)], 'yx-')

        start, end = 0.9, 1.0
        ax1.plot(xdata[np.floor(anz_samples*start):np.floor(anz_samples*end)], ydata[np.floor(anz_samples*start):np.floor(anz_samples*end)], 'rx-')


        ax1.legend(['Aceleration', 'Linear', 'Deceleration'])
        ax1.set_xlabel('x [m]')
        ax1.set_ylabel('y [m]')

        ##################################################

        plt.axis('equal')

        plt.tight_layout()
        plt.show()