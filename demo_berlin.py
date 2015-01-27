#!/usr/bin/env python2
#-*- coding: utf-8 -*-

from core import *

class Stuff(BaseScene):

    def __init__(self, profile):
        super(Stuff, self).__init__(profile)

        self.pose_home = {'p1': 0, 'p2': 0,
                          'p3': 0, 'p4': 0,
                          'p5': 0, 'p6': 0}

        self.pose_test = {'p1': np.radians(0), 'p2': np.radians(0),
                          'p3': np.radians(45), 'p4': np.radians(0),
                          'p5': np.radians(0), 'p6': np.radians(0)}

        self.hold_ball_start = {'p1': 0.48, 'p2': 0.92,
                                'p3': 0.00, 'p4': 1.26,
                                'p5': 0.00, 'p6': 0.80,
                                'p7': 0.36-np.pi/2} #np.pi -> 1


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

    def play_recorded_trajectory_goal(self):
        jtp_list = JTP.load_trajectory_goal('trajectory_goal_data/trajectory_goal.yaml')
        self.appendArms(jtp_list)

    def move_hold_ball_start(self, duration=8):
        self.appendArms(self.movePose(duration=duration, pose=self.inject_zero_velocity(self.hold_ball_start)))

    def tf_test_startpos(self, duration=20):
        self.appendTFZ(self.lin(0, duration))
        self.syncAllTF()

    def tf_test_z(self, steptime=20, distance=0.2, rotation=np.pi/4):
        self.appendTFZ(self.sin(0, np.pi*2, steptime)*distance)
        self.syncAllTF()

    def tf_test_roll(self, steptime=20, distance=0.2, rotation=np.pi/8):
        self.appendTFRoll(self.sin(0, np.pi*2, steptime)*rotation)
        self.syncAllTF()



    def fake(self, duration=10, steptime=10, distance=0.1, rotation=np.pi/8):

        tfh = TransformHelper()
        tfh.add_unknown(self.profile.tf_link_name, self.profile.tf_source_name, self.profile.tf_link_ofs)
        arm_left_pose = tfh.getTransformation(self.profile.tf_link_name, 'arm_left_7_link')
        arm_right_pose = tfh.getTransformation(self.profile.tf_link_name, 'arm_right_7_link')


        #appendAllTFLFncList = [self.appendTFLX, self.appendTFLY, self.appendTFLZ, self.appendTFLRoll, self.appendTFLPitch, self.appendTFLYaw]


        if arm_right_pose[3] < 0:
            arm_right_pose[3] = (arm_right_pose[3] + np.pi * 2) % (np.pi * 2)

        if arm_left_pose[3] < 0:
            arm_left_pose[3] = (arm_left_pose[3] + np.pi * 2) % (np.pi * 2)

        duration_start = 1
        self.appendTFLX(self.lin(arm_left_pose[0], duration_start))
        self.appendTFLY(self.lin(arm_left_pose[1], duration_start))
        self.appendTFLZ(self.lin(arm_left_pose[2], duration_start))
        self.appendTFLRoll(self.lin(arm_left_pose[3], duration_start))
        self.appendTFLPitch(self.lin(arm_left_pose[4], duration_start))
        self.appendTFLYaw(self.lin(arm_left_pose[5], duration_start))

        self.appendTFRX(self.lin(arm_right_pose[0], duration_start))
        self.appendTFRY(self.lin(arm_right_pose[1], duration_start))
        self.appendTFRZ(self.lin(arm_right_pose[2], duration_start))
        self.appendTFRRoll(self.lin(arm_right_pose[3], duration_start))
        self.appendTFRPitch(self.lin(arm_right_pose[4], duration_start))
        self.appendTFRYaw(self.lin(arm_right_pose[5], duration_start))

        self.syncTrackingTF()
        self.start_tracking_left(self.profile.tf_link_left_name)
        self.start_tracking_right(self.profile.tf_link_right_name)


        self.appendTFLX(self.acc(arm_left_pose[0], self.profile.tf_link_left_ofs[0], duration))
        self.appendTFLY(self.acc(arm_left_pose[1], self.profile.tf_link_left_ofs[1], duration))
        self.appendTFLZ(self.acc(arm_left_pose[2], self.profile.tf_link_left_ofs[2], duration))
        self.appendTFLRoll(self.acc(arm_left_pose[3], self.profile.tf_link_left_ofs[3], duration))
        self.appendTFLPitch(self.acc(arm_left_pose[4], self.profile.tf_link_left_ofs[4], duration))
        self.appendTFLYaw(self.acc(arm_left_pose[5], self.profile.tf_link_left_ofs[5], duration))

        self.appendTFRX(self.acc(arm_right_pose[0], self.profile.tf_link_right_ofs[0], duration))
        self.appendTFRY(self.acc(arm_right_pose[1], self.profile.tf_link_right_ofs[1], duration))
        self.appendTFRZ(self.acc(arm_right_pose[2], self.profile.tf_link_right_ofs[2], duration))
        self.appendTFRRoll(self.acc(arm_right_pose[3], self.profile.tf_link_right_ofs[3], duration))
        self.appendTFRPitch(self.acc(arm_right_pose[4], self.profile.tf_link_right_ofs[4], duration))
        self.appendTFRYaw(self.acc(arm_right_pose[5], self.profile.tf_link_right_ofs[5], duration))



        self.syncAllTF()
        for _ in range(1):
            self.appendTFZ(self.sin(0, np.pi*2, steptime)*distance)

        self.syncAllTF()

        self.appendTFRoll(self.sin(0, np.pi*2, steptime)*rotation)

        self.syncAllTF()


        duration_stop = 5
        self.appendTFZ(self.lin(0, duration_stop))


        self.syncTrackingTF()
        self.stop_tracking_left(self.profile.tf_link_left_name)
        self.stop_tracking_right(self.profile.tf_link_right_name)

        self.syncAllTF()


        duration_stop = 1
        self.appendTFZ(self.lin(0, duration_stop))


        self.syncAllTF()




    def tf_rviz_test(self, steptime=3, distance=0.2, rotation=np.pi/4):
        self.appendTFX(self.sin(0, np.pi*2, steptime)*distance)
        self.syncTF()
        self.appendTFY(self.sin(0, np.pi*2, steptime)*distance)
        self.syncTF()
        self.appendTFZ(self.sin(0, np.pi*2, steptime)*distance)
        self.syncTF()
        self.appendTFRoll(self.sin(0, np.pi*2, steptime)*rotation)
        self.syncTF()
        self.appendTFPitch(self.sin(0, np.pi*2, steptime)*rotation)
        self.syncTF()
        self.appendTFYaw(self.sin(0, np.pi*2, steptime)*rotation)
        self.syncTF()




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

    test = Stuff(cob4_2_profile)

    test.tf_test_z()

    masterTimeline = test

    sh = ServiceHandler()
    sh.add_service_callback('scenario/po1', demo.pose_boring_walk_front_back_c1, demo)
    sh.add_service_callback('scenario/po2', demo.pose_cheer_arms_up, demo)
    sh.add_service_callback('scenario/po3', demo.pose_cheer_turn, demo)
    sh.add_service_callback('scenario/sc1', demo.cheerTurn, demo)

    sh.add_service_callback('scenario/test1', test.move_goal, test)
    sh.add_service_callback('scenario/test2', test.move_vel, test)
    sh.add_service_callback('scenario/test3', test.move_combine_repeat, test)

    sh.add_service_callback('scenario/test4', test.play_recorded_trajectory_goal, test)

    sh.add_service_callback('scenario/test5', test.move_hold_ball_start, test)
    sh.add_service_callback('scenario/test6', test.tf_test_startpos, test)
    sh.add_service_callback('scenario/test7', [test.tf_test_startpos, test.tf_test_z, test.tf_test_startpos], test)
    sh.add_service_callback('scenario/test8', test.tf_test_roll, test)
    sh.add_service_callback('scenario/test9', test.fake, test)




    sh.startup(masterTimeline)
