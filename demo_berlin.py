#!/usr/bin/env python2
#-*- coding: utf-8 -*-

from core import *


if __name__ == '__main__':
    cob4_2_profile = Profile(rate=30, max_linear_velocity=0.7,
                             max_angular_velocity=2.7,
                             max_linear_acceleration=0.022,
                             max_angular_acceleration=0.074,
                             switch_vel_to_goal_timeout=0.7)
