#!/usr/bin/env python2
#-*- coding: utf-8 -*-
#
#Author: Björn Eistel
#Contact: <eistel@gmail.com>
#
# THIS SOURCE-CODE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED. IN NO
# EVENT WILL THE AUTHOR BE HELD LIABLE FOR ANY DAMAGES ARISING FROM THE USE OF THIS SOURCE-CODE.
# USE AT YOUR OWN RISK.

import matplotlib.pyplot as plt
from scipy import integrate
import numpy as np


class AutoFlow(object):
    def __init__(self):
        self.step = 1  # in s
        self.max_time = 200  # in s

        self.precision = 3

        self.designProg()
        self.doCalculations()

        self.doPlot()
        self.printBNRStyle()


    def designProg(self):

        self.accPrg = {
            'arm': np.array([[0, 0]]),
            'cross': np.array([[0, 0]]),
            'allgond': np.array([[0, 0]]),
            'oddgond': np.array([[0, 0]]),
            'evengond': np.array([[0, 0]]),

            'gond1': np.array([[0, 0]]),
            'gond2': np.array([[0, 0]]),
            'gond3': np.array([[0, 0]]),
            'gond4': np.array([[0, 0]]),
            'gond5': np.array([[0, 0]]),
            'gond6': np.array([[0, 0]]),
        }
        # 0.47123889803846897 rad / s -> max
        spin180_4_4 = np.array([[0, 0], [2, 0.4], [6, -0.4], [8, 0]])
        spin360_4_4 = self.superposeValues(spin180_4_4, spin180_4_4, 4)

        spin360_4_4_2 = self.superposeValues(spin180_4_4, spin180_4_4, 0)

        spin180_3_4 = np.array([[0, 0], [3, 0.176], [9, -0.176], [12, 0]])
        spin360_3_4 = self.superposeValues(spin180_3_4, spin180_3_4, 6)

        spin180_2_4 = np.array([[0, 0], [4, 0.1], [12, -0.1], [16, 0]])
        spin360_2_4 = self.superposeValues(spin180_2_4, spin180_2_4, 4)

        nspin180_4_4 = np.array([[0, 0], [2, -0.4], [6, 0.4], [8, 0]])
        nspin360_4_4 = self.superposeValues(nspin180_4_4, nspin180_4_4, 4)

        nspin360_4_4_2 = self.superposeValues(nspin180_4_4, nspin180_4_4, 0)

        nspin180_3_4 = np.array([[0, 0], [3, -0.176], [9, 0.176], [12, 0]])
        nspin360_3_4 = self.superposeValues(nspin180_3_4, nspin180_3_4, 6)

        nspin180_2_4 = np.array([[0, 0], [4, -0.1], [12, 0.1], [16, 0]])
        nspin360_2_4 = self.superposeValues(nspin180_2_4, nspin180_2_4, 4)

        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], spin180_2_4, 0)
        self.accPrg['allgond'] = self.superposeValues(self.accPrg['allgond'], spin180_2_4, 0)
        self.accPrg['cross'] = self.superposeValues(self.accPrg['cross'], spin360_2_4, 16)
        self.accPrg['cross'] = self.superposeValues(self.accPrg['cross'], spin360_3_4, 18)
        self.accPrg['cross'] = self.superposeValues(self.accPrg['cross'], spin360_4_4, 24)
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], spin180_4_4, 28)
        self.accPrg['allgond'] = self.superposeValues(self.accPrg['allgond'], spin180_4_4, 28)

        ts = 36
        self.accPrg['gond1'] = self.superposeValues(self.accPrg['gond1'], spin360_4_4_2, ts)
        self.accPrg['gond2'] = self.superposeValues(self.accPrg['gond2'], spin360_4_4_2, ts+1)
        self.accPrg['gond3'] = self.superposeValues(self.accPrg['gond3'], spin360_4_4_2, ts+2)
        self.accPrg['gond4'] = self.superposeValues(self.accPrg['gond4'], spin360_4_4_2, ts+3)
        self.accPrg['gond5'] = self.superposeValues(self.accPrg['gond5'], spin360_4_4_2, ts+4)
        self.accPrg['gond6'] = self.superposeValues(self.accPrg['gond6'], spin360_4_4_2, ts+5)

        ts = 50
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], nspin180_4_4, ts)
        self.accPrg['oddgond'] = self.superposeValues(self.accPrg['oddgond'], nspin180_4_4, ts)
        self.accPrg['evengond'] = self.superposeValues(self.accPrg['evengond'], spin180_4_4, ts)

        ts = 58
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], spin180_4_4, ts)
        self.accPrg['oddgond'] = self.superposeValues(self.accPrg['oddgond'], spin180_4_4, ts)
        self.accPrg['evengond'] = self.superposeValues(self.accPrg['evengond'], nspin180_4_4, ts)

        ts = 64
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], nspin180_4_4, ts)
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], nspin180_4_4, ts+4)
        self.accPrg['cross'] = self.superposeValues(self.accPrg['cross'], nspin360_4_4, ts)

        ts = 72
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], nspin180_4_4, ts)
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], nspin180_4_4, ts+4)
        self.accPrg['cross'] = self.superposeValues(self.accPrg['cross'], nspin360_4_4_2, ts)
        self.accPrg['cross'] = self.superposeValues(self.accPrg['cross'], nspin360_4_4_2, ts+4)

        ts = 84
        self.accPrg['gond1'] = self.superposeValues(self.accPrg['gond1'], nspin360_4_4_2, ts)
        self.accPrg['gond2'] = self.superposeValues(self.accPrg['gond2'], nspin360_4_4_2, ts+1)
        self.accPrg['gond3'] = self.superposeValues(self.accPrg['gond3'], nspin360_4_4_2, ts+2)
        self.accPrg['gond4'] = self.superposeValues(self.accPrg['gond4'], nspin360_4_4_2, ts+3)
        self.accPrg['gond5'] = self.superposeValues(self.accPrg['gond5'], nspin360_4_4_2, ts+4)
        self.accPrg['gond6'] = self.superposeValues(self.accPrg['gond6'], nspin360_4_4_2, ts+5)
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], nspin180_4_4, ts)
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], nspin180_4_4, ts+5)
        self.accPrg['cross'] = self.superposeValues(self.accPrg['cross'], nspin360_4_4, ts)

        ts = 88
        self.accPrg['gond1'] = self.superposeValues(self.accPrg['gond1'], nspin360_4_4_2, ts)
        self.accPrg['gond2'] = self.superposeValues(self.accPrg['gond2'], nspin360_4_4_2, ts+1)
        self.accPrg['gond3'] = self.superposeValues(self.accPrg['gond3'], nspin360_4_4_2, ts+2)
        self.accPrg['gond4'] = self.superposeValues(self.accPrg['gond4'], nspin360_4_4_2, ts+3)
        self.accPrg['gond5'] = self.superposeValues(self.accPrg['gond5'], nspin360_4_4_2, ts+4)
        self.accPrg['gond6'] = self.superposeValues(self.accPrg['gond6'], nspin360_4_4_2, ts+5)
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], nspin180_4_4, ts)
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], nspin180_4_4, ts+5)
        self.accPrg['cross'] = self.superposeValues(self.accPrg['cross'], nspin360_4_4, ts)

        ts = 101
        self.accPrg['gond1'] = self.superposeValues(self.accPrg['gond1'], spin360_4_4_2, ts)
        self.accPrg['gond2'] = self.superposeValues(self.accPrg['gond2'], spin360_4_4_2, ts+1)
        self.accPrg['gond3'] = self.superposeValues(self.accPrg['gond3'], spin360_4_4_2, ts+2)
        self.accPrg['gond4'] = self.superposeValues(self.accPrg['gond4'], spin360_4_4_2, ts+3)
        self.accPrg['gond5'] = self.superposeValues(self.accPrg['gond5'], spin360_4_4_2, ts+4)
        self.accPrg['gond6'] = self.superposeValues(self.accPrg['gond6'], spin360_4_4_2, ts+5)
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], spin180_4_4, ts)
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], spin180_4_4, ts+5)
        self.accPrg['cross'] = self.superposeValues(self.accPrg['cross'], nspin360_4_4, ts)

        ts = 114
        self.accPrg['gond1'] = self.superposeValues(self.accPrg['gond1'], spin360_4_4_2, ts)
        self.accPrg['gond2'] = self.superposeValues(self.accPrg['gond2'], spin360_4_4_2, ts+1)
        self.accPrg['gond3'] = self.superposeValues(self.accPrg['gond3'], spin360_4_4_2, ts+2)
        self.accPrg['gond4'] = self.superposeValues(self.accPrg['gond4'], spin360_4_4_2, ts+3)
        self.accPrg['gond5'] = self.superposeValues(self.accPrg['gond5'], spin360_4_4_2, ts+4)
        self.accPrg['gond6'] = self.superposeValues(self.accPrg['gond6'], spin360_4_4_2, ts+5)
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], spin180_4_4, ts)
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], spin180_4_4, ts+5)
        self.accPrg['cross'] = self.superposeValues(self.accPrg['cross'], nspin360_4_4, ts)

        ts = 122
        self.accPrg['gond1'] = self.superposeValues(self.accPrg['gond1'], spin360_4_4_2, ts)
        self.accPrg['gond2'] = self.superposeValues(self.accPrg['gond2'], spin360_4_4_2, ts+1)
        self.accPrg['gond3'] = self.superposeValues(self.accPrg['gond3'], spin360_4_4_2, ts+2)
        self.accPrg['gond4'] = self.superposeValues(self.accPrg['gond4'], spin360_4_4_2, ts+3)
        self.accPrg['gond5'] = self.superposeValues(self.accPrg['gond5'], spin360_4_4_2, ts+4)
        self.accPrg['gond6'] = self.superposeValues(self.accPrg['gond6'], spin360_4_4_2, ts+5)
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], spin180_4_4, ts)
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], spin180_4_4, ts+5)
        self.accPrg['cross'] = self.superposeValues(self.accPrg['cross'], nspin360_4_4, ts)

        ts = 132
        self.accPrg['gond1'] = self.superposeValues(self.accPrg['gond1'], nspin360_4_4_2, ts)
        self.accPrg['gond2'] = self.superposeValues(self.accPrg['gond2'], nspin360_4_4_2, ts+1)
        self.accPrg['gond3'] = self.superposeValues(self.accPrg['gond3'], nspin360_4_4_2, ts+2)
        self.accPrg['gond4'] = self.superposeValues(self.accPrg['gond4'], nspin360_4_4_2, ts+3)
        self.accPrg['gond5'] = self.superposeValues(self.accPrg['gond5'], nspin360_4_4_2, ts+4)
        self.accPrg['gond6'] = self.superposeValues(self.accPrg['gond6'], nspin360_4_4_2, ts+5)
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], spin180_4_4, ts)
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], spin180_4_4, ts+5)
        self.accPrg['cross'] = self.superposeValues(self.accPrg['cross'], spin360_4_4, ts)

        ts = 135
        self.accPrg['gond1'] = self.superposeValues(self.accPrg['gond1'], nspin360_4_4_2, ts)
        self.accPrg['gond2'] = self.superposeValues(self.accPrg['gond2'], nspin360_4_4_2, ts+1)
        self.accPrg['gond3'] = self.superposeValues(self.accPrg['gond3'], nspin360_4_4_2, ts+2)
        self.accPrg['gond4'] = self.superposeValues(self.accPrg['gond4'], nspin360_4_4_2, ts+3)
        self.accPrg['gond5'] = self.superposeValues(self.accPrg['gond5'], nspin360_4_4_2, ts+4)
        self.accPrg['gond6'] = self.superposeValues(self.accPrg['gond6'], nspin360_4_4_2, ts+5)
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], spin180_4_4, ts)
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], spin180_4_4, ts+5)
        self.accPrg['cross'] = self.superposeValues(self.accPrg['cross'], spin360_4_4, ts)

        ts = 138
        self.accPrg['gond1'] = self.superposeValues(self.accPrg['gond1'], nspin360_4_4_2, ts)
        self.accPrg['gond2'] = self.superposeValues(self.accPrg['gond2'], nspin360_4_4_2, ts+1)
        self.accPrg['gond3'] = self.superposeValues(self.accPrg['gond3'], nspin360_4_4_2, ts+2)
        self.accPrg['gond4'] = self.superposeValues(self.accPrg['gond4'], nspin360_4_4_2, ts+3)
        self.accPrg['gond5'] = self.superposeValues(self.accPrg['gond5'], nspin360_4_4_2, ts+4)
        self.accPrg['gond6'] = self.superposeValues(self.accPrg['gond6'], nspin360_4_4_2, ts+5)
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], spin180_4_4, ts)
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], spin180_4_4, ts+5)
        self.accPrg['cross'] = self.superposeValues(self.accPrg['cross'], spin360_4_4, ts)

        ts = 141
        self.accPrg['gond1'] = self.superposeValues(self.accPrg['gond1'], nspin360_4_4_2, ts)
        self.accPrg['gond2'] = self.superposeValues(self.accPrg['gond2'], nspin360_4_4_2, ts+1)
        self.accPrg['gond3'] = self.superposeValues(self.accPrg['gond3'], nspin360_4_4_2, ts+2)
        self.accPrg['gond4'] = self.superposeValues(self.accPrg['gond4'], nspin360_4_4_2, ts+3)
        self.accPrg['gond5'] = self.superposeValues(self.accPrg['gond5'], nspin360_4_4_2, ts+4)
        self.accPrg['gond6'] = self.superposeValues(self.accPrg['gond6'], nspin360_4_4_2, ts+5)
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], spin180_4_4, ts)
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], spin180_4_4, ts+5)
        self.accPrg['cross'] = self.superposeValues(self.accPrg['cross'], spin360_4_4, ts)

        ts = 144
        self.accPrg['gond1'] = self.superposeValues(self.accPrg['gond1'], nspin360_4_4_2, ts)
        self.accPrg['gond2'] = self.superposeValues(self.accPrg['gond2'], nspin360_4_4_2, ts+1)
        self.accPrg['gond3'] = self.superposeValues(self.accPrg['gond3'], nspin360_4_4_2, ts+2)
        self.accPrg['gond4'] = self.superposeValues(self.accPrg['gond4'], nspin360_4_4_2, ts+3)
        self.accPrg['gond5'] = self.superposeValues(self.accPrg['gond5'], nspin360_4_4_2, ts+4)
        self.accPrg['gond6'] = self.superposeValues(self.accPrg['gond6'], nspin360_4_4_2, ts+5)
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], spin180_4_4, ts)
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], spin180_4_4, ts+5)
        self.accPrg['cross'] = self.superposeValues(self.accPrg['cross'], spin360_4_4, ts)

        ts = 148
        self.accPrg['gond1'] = self.superposeValues(self.accPrg['gond1'], spin360_4_4_2, ts)
        self.accPrg['gond2'] = self.superposeValues(self.accPrg['gond2'], spin360_4_4_2, ts+1)
        self.accPrg['gond3'] = self.superposeValues(self.accPrg['gond3'], spin360_4_4_2, ts+2)
        self.accPrg['gond4'] = self.superposeValues(self.accPrg['gond4'], spin360_4_4_2, ts+3)
        self.accPrg['gond5'] = self.superposeValues(self.accPrg['gond5'], spin360_4_4_2, ts+4)
        self.accPrg['gond6'] = self.superposeValues(self.accPrg['gond6'], spin360_4_4_2, ts+5)
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], nspin180_4_4, ts)
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], nspin180_4_4, ts+5)
        self.accPrg['cross'] = self.superposeValues(self.accPrg['cross'], spin360_4_4, ts)

        ts = 161
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], nspin180_4_4, ts)
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], nspin180_4_4, ts+4)
        self.accPrg['cross'] = self.superposeValues(self.accPrg['cross'], nspin180_2_4, ts)
        self.accPrg['cross'] = self.superposeValues(self.accPrg['cross'], nspin180_2_4, ts+8)

        ts = 169
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], nspin180_4_4, ts)
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], nspin180_4_4, ts+4)

        ts = 177
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], nspin180_4_4, ts)
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], nspin180_4_4, ts+4)

        ts = 185
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], nspin180_4_4, ts)
        self.accPrg['arm'] = self.superposeValues(self.accPrg['arm'], nspin180_4_4, ts+4)

        ts = 161
        self.accPrg['gond1'] = self.superposeValues(self.accPrg['gond1'], spin360_4_4, ts)
        self.accPrg['gond2'] = self.superposeValues(self.accPrg['gond2'], spin360_4_4, ts+4)
        self.accPrg['gond3'] = self.superposeValues(self.accPrg['gond3'], spin360_4_4, ts+8)
        self.accPrg['gond4'] = self.superposeValues(self.accPrg['gond4'], spin360_4_4, ts+12)
        self.accPrg['gond5'] = self.superposeValues(self.accPrg['gond5'], spin360_4_4, ts+16)
        self.accPrg['gond6'] = self.superposeValues(self.accPrg['gond6'], spin360_4_4, ts+20)

        self.mergeGlobalSpecc()


    def mergeGlobalSpecc(self):
        for i in range(1, 7):
            self.accPrg['gond%d'%i] = self.superposeValues(self.accPrg['gond%d'%i],
                                                           self.accPrg['allgond'], 0)

        for i in range(1, 7, 2):
            self.accPrg['gond%d'%i] = self.superposeValues(self.accPrg['gond%d'%i],
                                                           self.accPrg['oddgond'], 0)

        for i in range(2, 7, 2):
            self.accPrg['gond%d'%i] = self.superposeValues(self.accPrg['gond%d'%i],
                                                           self.accPrg['evengond'], 0)


    def doCalculations(self):
        self.calcedValues = {}
        for k, v in self.accPrg.iteritems():
            xdata, ydata = self.adjustToMaxTime(v)
            vel, dist = self.getAccVelDist(xdata, ydata)
            self.calcedValues[k] = {'x': xdata, 'acc': ydata, 'vel': vel, 'dist': dist}


    def printBNRStyle(self):
        print '\n'*5
        print '\t\tmaxsize : UINT := %d;'%(len(self.calcedValues['arm']['x'][::16])-1)
        print '\t\tstepsize : REAL := %.2f;'%self.step
        for k, v in self.calcedValues.iteritems():
            if k in ['evengond', 'allgond', 'oddgond']:
                continue
            self.printBNROutput(k, v['vel'][::16])
        print '\n'*5


    def getAccVelDist(self, datax, datay):
        vel = integrate.cumtrapz(datay, datax, initial=0)
        dist = integrate.cumtrapz(vel, datax, initial=0)
        return vel, dist


    def superposeValues(self, data, new, shift):
        newc = new.copy()
        newc.ravel()[::2] += shift

        dx, dy = self.adjustToMaxTime(data)
        nx, ny = self.adjustToMaxTime(newc)
        y = dy+ny

        l = []
        for i in range(len(dx)):
            l.append([dx[i], y[i]])
        k = np.array(l)

        return k


    def adjustToMaxTime(self, data):
        ydata = data.ravel('C')[1::2]
        xdata = data.ravel('C')[::2]

        xspace = np.linspace(0, self.max_time, (self.max_time/self.step*16)+1)
        yspace = np.interp(xspace, xdata, ydata)

        return xspace, yspace


    def doPlot(self):
        legendsize = 10
        alpha = 0.7
        xlim = self.max_time+35
        hinch = 8.3
        winch = 12.5
        dpi = 120

        fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(nrows=3, ncols=1)
        ax1, ax2, ax3 = self.ax1, self.ax2, self.ax3

        acccap = 'Beschleunigung'
        velcap = 'Geschwindigkeit'
        distcap = 'Strecke'

        acclblx = 't [s]'
        vellblx = 't [s]'
        distlblx = 't [s]'

        acclbly = 'alpha [rad / s^2]'
        vellbly = 'omega [rad / s]'
        distlbly = 'phi [rad]'

        ax1.set_title(acccap)
        ax1.set_xlabel(acclblx)
        ax1.set_ylabel(acclbly)

        ax2.set_title(velcap)
        ax2.set_xlabel(vellblx)
        ax2.set_ylabel(vellbly)

        ax3.set_title(distcap)
        ax3.set_xlabel(distlblx)
        ax3.set_ylabel(distlbly)

        name = 'Arm'
        ax1.plot(self.calcedValues['arm']['x'], self.calcedValues['arm']['acc'], label=name,
                 alpha=alpha)
        ax2.plot(self.calcedValues['arm']['x'], self.calcedValues['arm']['vel'], label=name,
                 alpha=alpha)
        ax3.plot(self.calcedValues['arm']['x'], self.calcedValues['arm']['dist'], label=name,
                 alpha=alpha)

        name = 'Gondel-Kreuz'
        ax1.plot(self.calcedValues['cross']['x'], self.calcedValues['cross']['acc'], label=name,
                 alpha=alpha)
        ax2.plot(self.calcedValues['cross']['x'], self.calcedValues['cross']['vel'], label=name,
                 alpha=alpha)
        ax3.plot(self.calcedValues['cross']['x'], self.calcedValues['cross']['dist'], label=name,
                 alpha=alpha)

        ax1.legend(prop={'size': legendsize})
        ax2.legend(prop={'size': legendsize})
        ax3.legend(prop={'size': legendsize})

        ax1.set_xlim(right=xlim)
        ax2.set_xlim(right=xlim)
        ax3.set_xlim(right=xlim)

        plt.tight_layout()
        fig.set_size_inches(winch, hinch)
        plt.savefig(
            '/home/acuda/studium/Semester5/Steuerungstechnik/projekt/grafc/autoprog_arm_cross.png',
            dpi=dpi)

        ######################################################################
        ######################################################################

        fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(nrows=3, ncols=1)
        ax1, ax2, ax3 = self.ax1, self.ax2, self.ax3

        ax1.set_title(acccap)
        ax1.set_xlabel(acclblx)
        ax1.set_ylabel(acclbly)

        ax2.set_title(velcap)
        ax2.set_xlabel(vellblx)
        ax2.set_ylabel(vellbly)

        ax3.set_title(distcap)
        ax3.set_xlabel(distlblx)
        ax3.set_ylabel(distlbly)

        for i in range(1, 7):
            name = 'Gondel %d'%i
            data = self.calcedValues['gond%d'%i]
            ax1.plot(data['x'], data['acc'], label=name, alpha=alpha)
            ax2.plot(data['x'], data['vel'], label=name, alpha=alpha)
            ax3.plot(data['x'], data['dist'], label=name, alpha=alpha)

        ax1.legend(prop={'size': legendsize})
        ax2.legend(prop={'size': legendsize})
        ax3.legend(prop={'size': legendsize})

        ax1.set_xlim(right=xlim)
        ax2.set_xlim(right=xlim)
        ax3.set_xlim(right=xlim)

        plt.tight_layout()
        fig.set_size_inches(winch, hinch)
        plt.savefig(
            '/home/acuda/studium/Semester5/Steuerungstechnik/projekt/grafc/autoprog_gond.png',
            dpi=dpi)
        plt.show()


    def printBNROutput(self, name, datax):
        formatValues = '%d(%.'+str(self.precision)+'f)'
        formatStr = '\t\t%s : ARRAY[0..%d] OF REAL := [%s];'

        formatValueResult = list()
        lastvalue = datax[0]
        cntr = 0
        for i, x in enumerate(datax):
            if lastvalue != x or len(datax)-1 == i:
                if len(datax)-1 == i:
                    cntr += 1
                value = formatValues%(cntr, lastvalue)
                if i != 0 and (i%49) == 0:
                    value = '\n'+'\t'*6+value
                formatValueResult.append(value)
                lastvalue = x
                cntr = 0
            cntr += 1

        formatResult = formatStr%(name, len(datax)-1, ', '.join(formatValueResult))
        print formatResult


if __name__ == "__main__":
    af = AutoFlow()