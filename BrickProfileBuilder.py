#!/usr/bin/env python2
#-*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import numpy as np
import time
import sys
from scipy import integrate

class Profile(object):
    def __init__(self, rate, max_linear_velocity, max_angular_velocity, max_linear_acceleration, max_angular_acceleration):
        self.rate = float(rate)  # [Hz]
        self.sample_time = 1.0 / rate  # [s]

        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        self.max_linear_acceleration = max_linear_acceleration
        self.max_angular_acceleration = max_angular_acceleration



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
        tdata_acc = np.linspace(0, profile.sample_time * anz_samples_acc, anz_samples_acc)

        anz_samples_dec = self.calc_samples(duration * dec_percentage)
        tdata_dec = np.linspace(0, profile.sample_time * anz_samples_dec, anz_samples_dec)


        th_max = -2.0*phi / (duration * (acc_percentage + dec_percentage - 2.0))
        th_t_acc = th_max * (-np.cos(np.pi * tdata_acc / (duration * acc_percentage)) + 1.0) / 2.0
        th_t_lin = self.lin(velocity=th_max, duration=duration * (1.0-(acc_percentage + dec_percentage)))
        th_t_dec = th_max * (-np.cos(np.pi * tdata_dec / (duration * dec_percentage)) + 1.0) / 2.0

        th_t = np.append(th_t_acc, th_t_lin)
        th_t = np.append(th_t, th_t_dec[::-1])

        v_t = th_t * radius

        return v_t, th_t

class Timeline(object):
    def __init__(self):
        self.TLX = np.array([], np.float)
        self.TLY = np.array([], np.float)
        self.TLTH = np.array([], np.float)
        self.SECTIONS = list()

    def appendX(self, data):
        self.TLX = np.append(self.TLX, data)

    def appendY(self, data):
        self.TLY = np.append(self.TLY, data)

    def appendTH(self, data):
        self.TLTH = np.append(self.TLTH, data)

    def syncTimeline(self):
        self.TLX, self.TLY, self.TLTH = self._evenMaxSamples(self.TLX, self.TLY, self.TLTH)

    def _evenMaxSamples(self, *args):
        args = list(args)
        max_samples = max(map(len, args))
        for idx, sample_line in enumerate(args):
            remaining_samples = max_samples - len(sample_line)
            fill_data = np.zeros((remaining_samples), np.float)
            args[idx] = np.append(sample_line, fill_data)
        return args

    def __len__(self):
        return max(map(len, [self.TLX, self.TLY, self.TLTH]))

    def new_section(self, name='', sync_timelines=True):
        if sync_timelines:
            self.syncTimeline()
        self.SECTIONS.append({'name': name, 'at_sample': len(self)})



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

        for step in range(max([len(timeline.TLX), len(timeline.TLY), len(timeline.TLTH)])):
            twist = Twist()
            if step < len(timeline.TLX):
                twist.linear.x = timeline.TLX[step]
            if step < len(timeline.TLY):
                twist.linear.y = timeline.TLY[step]
            if step < len(timeline.TLTH):
                twist.angular.z = timeline.TLTH[step]

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


    profile = Profile(rate=100, max_linear_velocity=0.4, max_angular_velocity=0.6,
                      max_linear_acceleration=0.01, max_angular_acceleration=0.01)
    bricks = Bricks(profile)

    timeline = Timeline()
    timeline.appendX(bricks.lin(duration=3, velocity=0))


    # KREISBAHN
    ############

    timeline.new_section('test lin')
    timeline.appendX(bricks.lin_acc(velocity_start=0, velocity_lin=0.4, velocity_end=0, acc_percentage=0.25, dec_percentage=0.25, duration=3))
    timeline.syncTimeline()

    timeline.new_section('test circular_path')
    tlx, tlth = bricks.circular_path(radius=0.5, phi=np.pi/2, duration=4)
    timeline.appendX(tlx)
    timeline.appendTH(tlth)





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

    timeline.syncTimeline()


    TLX = timeline.TLX
    TLY = timeline.TLY
    TLTH = timeline.TLTH

    if is_ros:
        bridge = ROSBridge(profile, fakerun=is_fakerun)
        bridge.exec_timeline(timeline)

    if is_plot:
        import matplotlib.pyplot as plt
        import matplotlib.collections as collections
        import scipy

        max_samples = max([len(TLX), len(TLY), len(TLTH)])
        tdata = np.linspace(0, profile.sample_time * max_samples, max_samples)


        # distance / absphi
        TLXD = np.array([0], np.float)
        TLYD = np.array([0], np.float)
        TLTHD = np.array([0], np.float)

        TLXD = np.append(TLXD, integrate.cumtrapz(TLX, tdata))
        TLYD = np.append(TLYD, integrate.cumtrapz(TLY, tdata))
        TLTHD = np.append(TLTHD, integrate.cumtrapz(TLTH, tdata))

        # acceleration
        TLXA = np.array([0], np.float)
        TLYA = np.array([0], np.float)
        TLTHA = np.array([0], np.float)

        TLXA = np.append(TLXA, scipy.diff(TLX))
        TLYA = np.append(TLYA, scipy.diff(TLY))
        TLTHA = np.append(TLTHA, scipy.diff(TLTH))

        if plot_profile:
            fig, (ax1, ax2, ax3) = plt.subplots(nrows=3, ncols=1)
            legend = ['x', 'y', '$\\theta$']

            ax1.plot(tdata, TLXD)
            ax1.plot(tdata, TLYD)
            ax1.plot(tdata, TLTHD)
            ax1.legend(legend, loc=2)
            ax1.set_title('Distance')
            ax1.grid(True)

            ##################################################

            accwarn1 = np.abs(TLX) > profile.max_linear_velocity
            accwarn2 = np.abs(TLY) > profile.max_linear_velocity
            accwarn3 = np.abs(TLTH) > profile.max_angular_velocity
            accwarn = np.logical_or(accwarn1, accwarn2)
            accwarn = np.logical_or(accwarn, accwarn3)

            collection = collections.BrokenBarHCollection.span_where(tdata, ymin=-1, ymax=1, where=accwarn, facecolor='black', alpha=0.2)
            ax2.add_collection(collection)

            ax2.plot(tdata, TLX)
            ax2.plot(tdata, TLY)
            ax2.plot(tdata, TLTH)
            ax2.legend(legend, loc=2)
            ax2.set_title('Velocity')
            ax2.grid(True)

            ##################################################

            ax3.plot(tdata, TLXA)
            ax3.plot(tdata, TLYA)
            ax3.plot(tdata, TLTHA)
            ax3.legend(legend, loc=2)
            ax3.set_title('Acceleration')
            ax3.grid(True)

            accwarn1 = np.abs(TLXA) > profile.max_linear_acceleration
            accwarn2 = np.abs(TLYA) > profile.max_linear_acceleration
            accwarn3 = np.abs(TLTHA) > profile.max_angular_acceleration
            accwarn = np.logical_or(accwarn1, accwarn2)
            accwarn = np.logical_or(accwarn, accwarn3)

            collection = collections.BrokenBarHCollection.span_where(tdata, ymin=-1, ymax=1, where=accwarn, facecolor='black', alpha=0.2)
            ax3.add_collection(collection)

            plt.tight_layout()
        ##################################################
        # MAP
        ##################################################

        if plot_map:
            fig, (ax1) = plt.subplots(nrows=1, ncols=1)

            x = TLX * np.cos(TLTHD) + TLY * np.sin(TLTHD)
            y = TLY * np.cos(TLTHD) + TLX * np.sin(TLTHD)

            x = integrate.cumtrapz(x, tdata)
            y = integrate.cumtrapz(y, tdata)


            last_endpoint = 0
            color_scale = 0.5
            for idx, sec in enumerate(timeline.SECTIONS):
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

            accx = TLXA > 0
            decx = TLXA < 0

            accy = TLYA > 0
            decy = TLYA < 0

            accwarn1 = np.abs(TLXA) > profile.max_linear_acceleration
            accwarn2 = np.abs(TLYA) > profile.max_linear_acceleration
            accwarn3 = np.abs(TLTHA) > profile.max_angular_acceleration
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

            for i in range(len(x))[::int(profile.rate/2)]:
                text = '\n%s' % np.round(tdata[i], 1)
                ax1.text(x[i], y[i], text, fontsize=8, alpha=0.15)
                ax1.plot(x[i], y[i], '.', color=(0, 0, 0), alpha=0.15)


            plt.tight_layout()
            plt.axis('equal')


        ##################################################

        if plot_map or plot_profile:
            plt.show()