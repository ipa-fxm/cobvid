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
        signal = self.lin(velocity=-0.0, duration=2)
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
        jtp_list = JTP.load_trajectory_goal('trajectory_goal_data/trajectory_goal.yaml', 1.0)
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

        self.hold_ball_start = {'p1': 0.48, 'p2': 0.92,
                                'p3': 0.00, 'p4': 1.26,
                                'p5': 0.00, 'p6': 0.80,
                                'p7': 0.36-np.pi/2} #np.pi -> 1

    def move_hold_ball_start(self, duration=8):
        self.appendMimic('happy')
        self.appendArms(self.movePose(duration=duration, pose=self.inject_zero_velocity(self.hold_ball_start)))

    def hold_ball(self, duration=10):

        tfh = TransformHelper()
        tfh.add_unknown(self.profile.tf_link_name, self.profile.tf_source_name, self.profile.tf_link_ofs)
        arm_left_pose = tfh.getTransformation(self.profile.tf_link_name, 'arm_left_7_link')
        arm_right_pose = tfh.getTransformation(self.profile.tf_link_name, 'arm_right_7_link')


        #appendAllTFLFncList = [self.appendTFLX, self.appendTFLY, self.appendTFLZ, self.appendTFLRoll, self.appendTFLPitch, self.appendTFLYaw]


        if arm_right_pose[3] < 0:
            arm_right_pose[3] = (arm_right_pose[3] + np.pi * 2) % (np.pi * 2)

        if arm_left_pose[3] < 0:
            arm_left_pose[3] = (arm_left_pose[3] + np.pi * 2) % (np.pi * 2)


        #self.appendTFL(self.profile.tf_link_left_ofs)
        #self.appendTFL(self.profile.tf_link_right_ofs)

        self.appendTFL(arm_left_pose)
        self.appendTFR(arm_right_pose)
        self.appendTFZ(self.lin(0, duration=1))
        self.syncTimeline()




        '''
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
        '''
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


        duration_stop = 5
        self.appendTFZ(self.lin(0, duration_stop))



        self.syncTrackingTF()
        self.stop_tracking_left(self.profile.tf_link_left_name)
        self.stop_tracking_right(self.profile.tf_link_right_name)

        self.syncAllTF()


        duration_stop = 1
        self.appendTFZ(self.lin(0, duration_stop))


        self.syncAllTF()


    def scene_z(self, dotime=10, distance=0.1):
        self.syncTimeline()

        self.appendTFZ(self.sin(0, np.pi*2, dotime)*distance)

        self.syncTimeline()


    def part_start_tracking(self):
        self.syncTimeline()
        self.appendTFL(self.profile.tf_link_left_ofs)
        self.appendTFR(self.profile.tf_link_right_ofs)
        self.appendTFZ(self.lin(0, duration=1))
        self.syncTimeline()

        self.start_tracking_left(self.profile.tf_link_left_name)
        self.start_tracking_right(self.profile.tf_link_right_name)
        self.syncTimeline()

    def part_stop_tracking(self):
        self.syncTimeline()

        #FIXME: control settling time... should not be needed!!!
        self.appendTFZ(self.lin(0, duration=5))

        self.syncTimeline()

        self.stop_tracking_left(self.profile.tf_link_left_name)
        self.stop_tracking_right(self.profile.tf_link_right_name)

        self.appendTFZ(self.lin(0, duration=1))

        self.syncTimeline()

    def scene_roll(self, dotime=16, rotation=np.pi/8):
        self.syncTimeline()

        self.appendTFRoll(self.sin(0, np.pi*2, dotime)*rotation)

        self.syncTimeline()

    def record_tf_holdball(self):
        jtp_list = JTP.load_trajectory_goal('trajectory_goal_data/trajectory_goal_tf_holdball.yaml', 1.0)
        self.appendArms(jtp_list)

    def record_tf_z(self):
        jtp_list = JTP.load_trajectory_goal('trajectory_goal_data/trajectory_goal_tf_z.yaml', 1.0)
        self.appendArms(jtp_list)

    def record_tf_roll(self):
        jtp_list = JTP.load_trajectory_goal('trajectory_goal_data/trajectory_goal_tf_roll.yaml', 1.0)
        self.appendArms(jtp_list)

    def record_tf_cross(self):
        jtp_list = JTP.load_trajectory_goal('trajectory_goal_data/trajectory_goal_tf_cross.yaml', 1.0)
        self.appendArms(jtp_list)

    def record_tf_circ_8(self):
        jtp_list = JTP.load_trajectory_goal('trajectory_goal_data/trajectory_goal_tf_circ_8.yaml', 1.0)
        self.appendArms(jtp_list)

    def scene_cross_yz(self, duration=15, distance=0.1):

        shortduration = duration * 1.0/8.0
        longduration = duration * 2.0/8.0

        self.appendTFZ(self.acc(0, distance, shortduration))
        self.appendTFZ(self.acc(distance, -distance, longduration))
        self.appendTFZ(self.acc(-distance, 0, shortduration))

        self.syncTimeline()

        self.appendTFY(self.acc(0, distance, shortduration))
        self.appendTFY(self.acc(distance, -distance, longduration))
        self.appendTFY(self.acc(-distance, 0, shortduration))


    def scene_circ_8_yz(self, acc_time=2, duration_circle=6, distance=0.1):

        self.syncTimeline()
        sideofs = distance / np.sqrt(2)
        hypofs = distance * np.sqrt(2)

        acc_dec_data = self.acc(0, sideofs*2, acc_time*2)
        acc_data = acc_dec_data[:len(acc_dec_data)//2]
        #dec_data = acc_dec_data[len(acc_dec_data)//2:]

        self.appendTFZ(acc_data)
        self.appendTFY(acc_data)

        circ_z_top = self.sin(-np.pi/4, np.pi*5/4, duration_circle)*distance + hypofs
        circ_y_top = self.cos(-np.pi/4, np.pi*5/4, duration_circle)*distance
        self.appendTFZ(circ_z_top)
        self.appendTFY(circ_y_top)


        lin_data = np.linspace(sideofs, 0, sideofs / (acc_data[-1] - acc_data[-2]))
        self.appendTFZ(lin_data)
        self.appendTFY(lin_data*-1)

        ##### mirroring #####

        self.appendTFZ(lin_data[::-1]*-1)
        self.appendTFY(lin_data[::-1])

        self.appendTFZ(circ_z_top[::-1]*-1)
        self.appendTFY(circ_y_top[::-1]*-1)

        self.appendTFZ(acc_data[::-1]*-1)
        self.appendTFY(acc_data[::-1]*-1)



        self.syncTimeline()

    def scene_mp_circ_xy_yaw(self, duration=20.0, phi=np.pi/8, radian=0.2):
        th = np.array([])
        th = np.append(th, self.acc(0, phi, duration/4.0))
        th = np.append(th, self.acc(phi, -phi, duration/2.0))
        th = np.append(th, self.acc(-phi, 0, duration/4.0))

        self.appendTFYaw(th)
        self.appendTFY(np.sin(th)*-radian)
        self.appendTFX(radian-np.cos(th)*radian)

        self.syncTimeline()


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

    def led_color_demo(self, steptime=1.5, ntimes=2):
        self.syncTimeline()
        for _ in range(ntimes):

            self.appendLed(r=1, g=0, b=0, a=1, frequency=0, mode=1)
            fillData = [None]*self.calc_samples(steptime)
            self.LED.extend(fillData)

            self.appendLed(r=0, g=1, b=0, a=1, frequency=0, mode=1)
            fillData = [None]*self.calc_samples(steptime)
            self.LED.extend(fillData)

            self.appendLed(r=0, g=0, b=1, a=1, frequency=0, mode=1)
            fillData = [None]*self.calc_samples(steptime)
            self.LED.extend(fillData)

            self.appendLed(r=1, g=1, b=0, a=1, frequency=0, mode=1)
            fillData = [None]*self.calc_samples(steptime)
            self.LED.extend(fillData)

            self.appendLed(r=0, g=1, b=1, a=1, frequency=0, mode=1)
            fillData = [None]*self.calc_samples(steptime)
            self.LED.extend(fillData)

            self.appendLed(r=1, g=0, b=1, a=1, frequency=0, mode=1)
            fillData = [None]*self.calc_samples(steptime)
            self.LED.extend(fillData)

        self.appendLed()
        fillData = [None]*self.calc_samples(steptime)
        self.LED.extend(fillData)

        self.syncTimeline()


if __name__ == '__main__':
    cob4_2_profile = Profile(rate=50, max_linear_velocity=0.7,
                             max_angular_velocity=2.7,
                             max_linear_acceleration=0.022,
                             max_angular_acceleration=0.074,
                             switch_vel_to_goal_timeout=0.7)

    demo = DemoScene(cob4_2_profile)

    test = Stuff(cob4_2_profile)

    demo.scene_roll()

    masterTimeline = demo

    sh = ServiceHandler()

    sh.add_service_callback('scenario/po1', demo.move_hold_ball_start, demo)
    sh.add_service_callback('scenario/br1', demo.hold_ball, demo)

    sh.add_service_callback('scenario/sc1', [demo.part_start_tracking, demo.scene_z, demo.part_stop_tracking], demo)
    sh.add_service_callback('scenario/sc2', [demo.part_start_tracking, demo.scene_roll, demo.part_stop_tracking], demo)
    sh.add_service_callback('scenario/sc3', [demo.part_start_tracking, demo.scene_cross_yz, demo.part_stop_tracking], demo)
    sh.add_service_callback('scenario/sc4', [demo.part_start_tracking, demo.scene_circ_8_yz, demo.part_stop_tracking], demo)
    sh.add_service_callback('scenario/sc5', [demo.part_start_tracking, demo.scene_mp_circ_xy_yaw, demo.part_stop_tracking], demo)


    sh.add_service_callback('scenario/rec1', demo.record_tf_holdball, demo)
    sh.add_service_callback('scenario/rec2', demo.record_tf_z, demo)
    sh.add_service_callback('scenario/rec3', demo.record_tf_roll, demo)
    sh.add_service_callback('scenario/rec4', demo.record_tf_cross, demo)
    sh.add_service_callback('scenario/rec5', demo.record_tf_circ_8, demo)
    sh.add_service_callback('scenario/recx', [demo.record_tf_z, demo.record_tf_roll, demo.record_tf_cross, demo.record_tf_circ_8], demo)
    sh.add_service_callback('scenario/recnr', [demo.record_tf_z, demo.record_tf_cross, demo.record_tf_circ_8], demo)
    sh.add_service_callback('scenario/recxx', [demo.move_hold_ball_start, demo.record_tf_holdball, demo.record_tf_z, demo.record_tf_roll, demo.record_tf_cross, demo.record_tf_circ_8], demo)
    sh.add_service_callback('scenario/led', demo.led_color_demo, demo)

    #sh.add_service_callback('scenario/sc3', demo.scene_cross_yz, demo)
    #sh.add_service_callback('scenario/sc4', demo.scene_circ_8_yz, demo)
    #sh.add_service_callback('scenario/sc5', demo.scene_mp_circ_xy_yaw, demo)

    sh.add_service_callback('scenario/scx', [demo.part_start_tracking, demo.scene_roll, demo.scene_cross_yz, demo.scene_circ_8_yz, demo.scene_mp_circ_xy_yaw, demo.part_stop_tracking], demo)

    #sh.add_service_callback('scenario/po1', demo.pose_boring_walk_front_back_c1, demo)
    #sh.add_service_callback('scenario/po2', demo.pose_cheer_arms_up, demo)
    #sh.add_service_callback('scenario/po3', demo.pose_cheer_turn, demo)
    #sh.add_service_callback('scenario/sc1', demo.cheerTurn, demo)

    sh.add_service_callback('scenario/test1', test.move_goal, test)
    sh.add_service_callback('scenario/test2', test.move_vel, test)
    sh.add_service_callback('scenario/test3', test.move_combine_repeat, test)

    sh.add_service_callback('scenario/test4', test.play_recorded_trajectory_goal, test)
    sh.add_service_callback('scenario/test9', test.fake, test)




    sh.startup(masterTimeline)
