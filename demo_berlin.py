#!/usr/bin/env python2
#-*- coding: utf-8 -*-

from core import *

class TestStuff(BaseScene):

    def __init__(self, profile):
        super(TestStuff, self).__init__(profile)

        self.pose_home = {'p1': 0, 'p2': 0,
                          'p3': 0, 'p4': 0,
                          'p5': 0, 'p6': 0}

        self.pose_test = {'p1': np.radians(0), 'p2': np.radians(0),
                          'p3': np.radians(45), 'p4': np.radians(0),
                          'p5': np.radians(0), 'p6': np.radians(0)}

    def move_goal(self):
        self.appendArms(self.movePose(duration=2, pose=self.inject_zero_velocity(self.pose_test)))

    def move_vel(self):
        signal = self.lin(velocity=-0.3, duration=2)
        self.appendVelArmLeft(j3=signal)
        self.appendVelArmRight(j3=signal*-1)

    def move_combine_repeat(self, ntimes=3):
        for i in range(ntimes):
            self.move_goal()
            self.syncTimeline()
            self.move_vel()
            self.syncTimeline()
            self.appendSwitchVelToGoalTimeout()
            self.syncTimeline()



class DemoScene(BaseScene):
    def __init__(self, profile):
        super(DemoScene, self).__init__(profile)

    def pose_boring_walk_front_back_c1(self, dotime=12):
        self.appendArms(self.movePose(duration=dotime, pose=self.inject_zero_velocity(ArmMovement.pose_boring_walk_front_back_c1)))

    def pose_cheer_arms_up(self, dotime=8):
        self.appendArms(self.movePose(duration=dotime, pose=self.inject_zero_velocity(ArmMovement.pose_cheer_arms_up)))

    def pose_cheer_turn(self, dotime=8):
        self.appendArms(self.movePose(duration=dotime, pose=ArmMovement.pose_cheer_turn))

    def cheerTurn(self, arm_time=5, rot_phi=np.pi / 2, rot_time=10):

        self.appendArms(self.movePose(duration=arm_time, pose=ArmMovement.pose_cheer_arms_up))
        _, th = self.circular_path(0, rot_phi / 2, arm_time)
        self.appendTH(th)

        self.syncTimeline()

        self.appendArms(self.movePose(duration=rot_time*0.4, pose=self.inject_zero_velocity(ArmMovement.pose_cheer_turn)))
        self.appendArms(self.movePose(duration=rot_time*0.2, pose=self.inject_zero_velocity(ArmMovement.pose_cheer_turn)))
        self.appendArms(self.movePose(duration=rot_time*0.4, pose=ArmMovement.pose_cheer_arms_up))
        _, th = self.circular_path(0, -rot_phi, rot_time)
        self.appendTH(th)

        self.syncTimeline()

        self.appendArms(self.movePose(duration=arm_time, pose=self.inject_zero_velocity(ArmMovement.pose_boring_walk_front_back_c1)))
        _, th = self.circular_path(0, rot_phi / 2, arm_time)
        self.appendTH(th)

        self.syncTimeline()



if __name__ == '__main__':
    cob4_2_profile = Profile(rate=30, max_linear_velocity=0.7,
                             max_angular_velocity=2.7,
                             max_linear_acceleration=0.022,
                             max_angular_acceleration=0.074,
                             switch_vel_to_goal_timeout=0.7)

    demo = DemoScene(cob4_2_profile)

    test = TestStuff(cob4_2_profile)

    demo.cheerTurn()

    masterTimeline = demo

    sh = ServiceHandler()
    sh.add_service_callback('scenario/po1', demo.pose_boring_walk_front_back_c1, demo)
    sh.add_service_callback('scenario/po2', demo.pose_cheer_arms_up, demo)
    sh.add_service_callback('scenario/po3', demo.pose_cheer_turn, demo)
    sh.add_service_callback('scenario/sc1', demo.cheerTurn, demo)

    sh.add_service_callback('scenario/test1', test.move_goal, test)
    sh.add_service_callback('scenario/test2', test.move_vel, test)
    sh.add_service_callback('scenario/test3', test.move_combine_repeat, test)

    sh.startup(masterTimeline)
