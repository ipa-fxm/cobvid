#!/usr/bin/env python2
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

    def circular_path(self, radius, phi, duration, acc_percentage=None, dec_percentage=None):
        #TODO: implement acc_percentage, dec_percentage
        th_max = phi / (duration*0.9)

        anz_samples1 = self.calc_samples(duration*0.1)
        tdata1 = np.linspace(0, profile.sample_time * anz_samples1, anz_samples1)

        th_t = th_max / 2.0 * (-np.cos(10 * np.pi/duration * tdata1) + 1)
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

    is_fakerun = '-fakerun' in sys.argv
    is_plot = '-plot' in sys.argv
    if is_plot:
        idx = sys.argv.index('-plot') + 1
        plot_map = 'map' in sys.argv[idx:idx+2]
        plot_profile = 'profile' in sys.argv[idx:idx+2]


    is_ros = '-ros' in sys.argv


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

    TLX = np.append(TLX, bricks.lin(duration=3, velocity=0))
    # sync lines
    TLX, TLY, TLTH = bricks.evenMaxSamples(TLX, TLY, TLTH)

    TLX = np.append(TLX, bricks.acc(velocity_start=0, velocity_end=0.4, duration=1))
    TLX = np.append(TLX, bricks.lin(duration=2, velocity=0.4))
    TLX = np.append(TLX, bricks.acc(velocity_start=0.4, velocity_end=0, duration=1))

    TLX, TLY, TLTH = bricks.evenMaxSamples(TLX, TLY, TLTH)

    tlx, tlth = bricks.circular_path(radius=0.5, phi=np.pi/4, duration=4)
    TLX = np.append(TLX, tlx)
    TLTH = np.append(TLTH, tlth)

    # sync lines
    TLX, TLY, TLTH = bricks.evenMaxSamples(TLX, TLY, TLTH)

    TLX = np.append(TLX, bricks.acc(velocity_start=0, velocity_end=0.4, duration=1))
    TLX = np.append(TLX, bricks.lin(duration=1.5, velocity=0.4))
    TLX = np.append(TLX, bricks.acc(velocity_start=0.4, velocity_end=0, duration=1))


    TLX, TLY, TLTH = bricks.evenMaxSamples(TLX, TLY, TLTH)

    tlx, tlth = bricks.circular_path(radius=-0.5, phi=-np.pi*9/6, duration=10)
    TLX = np.append(TLX, tlx)
    TLTH = np.append(TLTH, tlth)

    # sync lines
    TLX, TLY, TLTH = bricks.evenMaxSamples(TLX, TLY, TLTH)

    TLX = np.append(TLX, bricks.acc(velocity_start=0, velocity_end=0.4, duration=1))
    TLX = np.append(TLX, bricks.lin(duration=1.5, velocity=0.4))
    TLX = np.append(TLX, bricks.acc(velocity_start=0.4, velocity_end=0, duration=1))


    TLX, TLY, TLTH = bricks.evenMaxSamples(TLX, TLY, TLTH)

    tlx, tlth = bricks.circular_path(radius=0.5, phi=np.pi*15/12, duration=8)
    TLX = np.append(TLX, tlx)
    TLTH = np.append(TLTH, tlth)

    # sync lines
    TLX, TLY, TLTH = bricks.evenMaxSamples(TLX, TLY, TLTH)

    TLX = np.append(TLX, bricks.acc(velocity_start=0, velocity_end=-0.4, duration=0.5))
    TLX = np.append(TLX, bricks.lin(duration=2.5, velocity=-0.4))
    TLX = np.append(TLX, bricks.acc(velocity_start=-0.4, velocity_end=0, duration=0.5))



    TLY = np.append(TLY, bricks.acc(velocity_start=0, velocity_end=-0.175, duration=0.5))
    TLY = np.append(TLY, bricks.lin_dist(duration=2, distance=-0.35))
    TLY = np.append(TLY, bricks.acc(velocity_start=-0.175, velocity_end=0, duration=0.5))




    # vor links
    #tlx, tlth = bricks.circular_path(radius=0.5, phi=np.pi/2, duration=6)
    #TLX = np.append(TLX, tlx)
    #TLTH = np.append(TLTH, tlth)

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

        TLX, TLY, TLTH = bricks.evenMaxSamples(TLX, TLY, TLTH)
        max_samples = max([len(TLX), len(TLY), len(TLTH)])
        tdata = np.linspace(0, profile.sample_time * max_samples, max_samples)

        TLXD = np.array([0], np.float)
        TLYD = np.array([0], np.float)
        TLTHD = np.array([0], np.float)

        TLXD = np.append(TLXD, integrate.cumtrapz(TLX, tdata))
        TLYD = np.append(TLYD, integrate.cumtrapz(TLY, tdata))
        TLTHD = np.append(TLTHD, integrate.cumtrapz(TLTH, tdata))

        if plot_profile:
            fig, (ax1, ax2, ax3) = plt.subplots(nrows=3, ncols=1)
            legend = ['x', 'y', '$\\theta$']

            ax1.plot(tdata, TLXD)
            ax1.plot(tdata, TLYD)
            ax1.plot(tdata, TLTHD)
            ax1.legend(legend)
            ax1.set_title('Distance')

            ##################################################


            ax2.plot(tdata, TLX)
            ax2.plot(tdata, TLY)
            ax2.plot(tdata, TLTH)
            ax2.legend(legend)
            ax2.set_title('Velocity')

            ##################################################


            TLXA = np.array([0], np.float)
            TLYA = np.array([0], np.float)
            TLTHA = np.array([0], np.float)

            TLXA = np.append(TLXA, scipy.diff(TLX))
            TLYA = np.append(TLYA, scipy.diff(TLY))
            TLTHA = np.append(TLTHA, scipy.diff(TLTH))

            ax3.plot(tdata, TLXA)
            ax3.plot(tdata, TLYA)
            ax3.plot(tdata, TLTHA)
            ax3.legend(legend)
            ax3.set_title('Acceleration')

        ##################################################
        # MAP
        ##################################################

        if plot_map:
            fig, (ax1) = plt.subplots(nrows=1, ncols=1)

            x = TLX * np.cos(TLTHD) + TLY * np.sin(TLTHD)
            y = TLY * np.cos(TLTHD) + TLX * np.sin(TLTHD)

            x = integrate.cumtrapz(x, tdata)
            y = integrate.cumtrapz(y, tdata)

            ax1.plot(x, y, '-', color=(0, 0, 0), alpha=0.5)
            ax1.set_xlabel('x [m]')
            ax1.set_ylabel('y [m]')
            ax1.legend(['Path Map'])

            plt.axis('equal')


        ##################################################

        if plot_map or plot_profile:
            plt.axis('equal')

            plt.tight_layout()
            plt.show()