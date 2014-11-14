#!/usr/bin/env python2
#-*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import numpy as np
import time
import sys
from scipy import integrate


class Plotter(object):
    def __init__(self, timeline, plot_profile=False, plot_map=False):

        import matplotlib.pyplot as plt
        import scipy

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
    class Dummy(object):
        pass

    def __init__(self, fakerun=False):
        if not fakerun:
            rospy.init_node('VID_TEST')
            self.pub = rospy.Publisher('/base_controller/command_direct', Twist)
        else:
            self.pub = ROSBridge.Dummy()
            self.pub.publish = self._print_fakerun

    def _print_fakerun(self, msg):
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
            time.sleep(timeline.profile.sample_time)


class Profile(object):
    def __init__(self, rate, max_linear_velocity, max_angular_velocity, max_linear_acceleration, max_angular_acceleration):
        self.rate = float(rate)  # [Hz]
        self.sample_time = 1.0 / rate  # [s]

        self.max_linear_velocity = max_linear_velocity
        self.max_angular_velocity = max_angular_velocity
        self.max_linear_acceleration = max_linear_acceleration
        self.max_angular_acceleration = max_angular_acceleration


class Timeline(object):
    def __init__(self, profile):
        self.profile = profile
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

    def appendReversePath(self):
        self.syncTimeline()
        self.appendX(self.TLX[::-1]*-1)
        self.appendY(self.TLY[::-1]*-1)
        self.appendTH(self.TLTH[::-1]*-1)

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


class BoringMovement(Timeline, Bricks):
    def __init__(self, profile):
        super(BoringMovement, self).__init__(profile)

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
        self.appendX(self.lin_acc(velocity_start=0, velocity_lin=0.7, velocity_end=0, acc_percentage=0.17, dec_percentage=0.4, duration=3))
        self.syncTimeline()

        self.appendReversePath()

    def test_speed_angular(self):
        self.appendX(self.lin(duration=2, velocity=0))
        self.syncTimeline()

        tlx, tlth = self.circular_path(radius=0, phi=np.pi, duration=1.8, acc_percentage=0.35, dec_percentage=0.35)
        self.appendX(tlx)
        self.appendTH(tlth)

        self.syncTimeline()

        self.appendReversePath()

    def test_speed_circula_path(self):
        self.appendX(self.lin(duration=2, velocity=0))
        self.syncTimeline()

        tlx, tlth = self.circular_path(radius=1.5, phi=np.pi/2, duration=5, acc_percentage=0.2, dec_percentage=0.2)
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


    def to_window(self):
        self.appendX(self.lin(duration=2, velocity=0))
        self.syncTimeline()

        self.new_section('annaehern')
        self.appendX(self.acc(velocity_start=0, velocity_end=0.31, duration=0.5))
        self.appendX(self.lin(0.31, 2))


        self.new_section('ausrichtung fenster')
        tlx, tlth = self.circular_path(radius=-1.25, phi=-np.pi/2, duration=10, acc_percentage=0, dec_percentage=0.5)
        self.appendX(tlx)
        self.appendTH(tlth)

        self.new_section('look')
        self.appendX(self.lin(duration=0.5, velocity=0))
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

        tlx, tlth = self.circular_path(radius=0, phi=-np.pi/4, duration=4, acc_percentage=0.6, dec_percentage=0.4)
        self.appendX(tlx)
        self.appendTH(tlth)
        self.syncTimeline()

        self.appendX(self.lin(duration=0.5, velocity=0))
        self.syncTimeline()

    def away_from_window(self):
        self.new_section('\nenfernen vom fenster')
        self.appendX(self.lin_acc(velocity_start=0, velocity_end=0.3, velocity_lin=-0.3, dec_percentage=0.5, duration=2))

        #self.appendTH(self.lin(velocity=0.8, duration=0.25))
        self.appendTH(self.lin_acc(velocity_start=0, velocity_end=0, velocity_lin=0.8, acc_percentage=0.2, dec_percentage=0.2, duration=2))

        #self.appendTH(self.circular_path(radius=0, ))

        #self.appendY(self.lin(velocity=0, duration=0.5))
        self.appendY(self.lin_acc(velocity_start=0, velocity_end=0, velocity_lin=0.3, acc_percentage=0.5, duration=1.5))


        self.syncTimeline()


        self.appendX(self.acc(velocity_start=0.3, velocity_end=0, duration=1))

        self.syncTimeline()

    def away_from_window2(self):
        self.new_section('\nenfernen vom fenster')

        scale = 3
        T = 1.5*scale

        self.appendX(self.lin(velocity=0, duration=T/4))
        self.appendX(self.sin_t(A=0.3, f=2.0/3/3/scale, phi=0, T=T*3/4))
        self.appendX(self.lin(0.3, T*1/4))
        self.appendY(self.sin_t(A=0.2/scale, f=2.0/3/scale, phi=0, T=T/2))
        self.appendTH(self.sin_t(A=0.7, f=1.0/3/scale, phi=0, T=T))

        self.syncTimeline()

    def away_from_window3(self):

        self.new_section('\nenfernen vom fenster')

        #self.appendX(self.acc(0, -1, 0.5))
        #self.appendY(self.acc(0, 1, 0.5))

        #self.appendY(self.sin(-np.pi, 0, 2))

        #self.appendX(self.sin(-np.pi, np.pi/4, 6)*0.2)
        #self.appendY(self.sin(0, np.pi, 6)*0.2)


        lin90_3, th90_3 = self.circular_path(radius=0.2, phi=np.pi/2, duration=3, acc_percentage=0.25, dec_percentage=0.25)
        lin90_6, th90_6 = self.circular_path(radius=0.5, phi=np.pi*5/8, duration=6, acc_percentage=0.35, dec_percentage=0.25)




        self.appendY(self.lin(velocity=0, duration=0.5))
        self.appendY(self.lin_acc(velocity_start=0, velocity_lin=0.15, velocity_end=0, acc_percentage=0.3, dec_percentage=0.3, duration=3.5))
        #self.appendY(lin90_3*2/3)

        self.appendTH(self.lin_acc(velocity_start=0, velocity_lin=0.55, velocity_end=0, acc_percentage=0.35, dec_percentage=0.25, duration=5))
        #self.appendTH(th90_6*1.1)



        #self.appendX(self.lin(velocity=0, duration=1.5))
        self.appendX(self.lin_acc(velocity_start=0, velocity_lin=-0.08, velocity_end=0, acc_percentage=0.7, dec_percentage=0.3, duration=1.5))
        self.appendX(self.lin_acc(velocity_start=0, velocity_lin=0.2, velocity_end=0, acc_percentage=0.15, dec_percentage=0.2, duration=5.5))
        #self.appendX(lin90_6*2/3)



        #self.appendX(self.lin(velocity=0, duration=3))
        #self.appendX(lin45*-1)


        #lin, th = self.circular_path(radius=-1, phi=np.pi/4, duration=3, acc_percentage=0, dec_percentage=0.5)
        #self.appendY(lin)
        #self.appendX(lin*-1)
        #self.appendTH(th)

        #self.new_section('\n90 grad')



        self.syncTimeline()

if __name__ == '__main__':

    is_fakerun = '-fakerun' in sys.argv
    is_plot = '-plot' in sys.argv
    if is_plot:
        idx = sys.argv.index('-plot') + 1
        plot_map = 'map' in sys.argv[idx:idx+2]
        plot_profile = 'profile' in sys.argv[idx:idx+2]
    is_ros = '-ros' in sys.argv

    cob3_3_profile = Profile(rate=100, max_linear_velocity=0.7, max_angular_velocity=2.7,
                             max_linear_acceleration=0.022, max_angular_acceleration=0.074)
    boring = BoringMovement(profile=cob3_3_profile)

    #boring.test_map()
    #boring.test_speed_linear()
    #boring.test_speed_angular()
    #boring.test_speed_circula_path()
    #boring.to_window()
    #boring.away_from_window()
    #boring.away_from_window2()
    boring.away_from_window3()
    #boring.appendReversePath()




    if is_ros:
        bridge = ROSBridge(fakerun=is_fakerun)
        bridge.exec_timeline(boring)

    if is_plot:
        Plotter(boring, plot_profile=plot_profile, plot_map=plot_map)
