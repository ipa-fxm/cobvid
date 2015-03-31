#!/usr/bin/env python2
#-*- coding: utf-8 -*-

from cobvid.core import *

class StuffToTest(BaseScene):
    def __init__(self, profile):
            super(StuffToTest, self).__init__(profile)

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
        #self.appendX(self.lin(duration=2, velocity=0))
        self.appendX(self.lin_acc(velocity_start=0, velocity_lin=0.7, velocity_end=0, acc_percentage=0.25, dec_percentage=0.25, duration=2))
        self.syncTimeline()

        #self.appendReversePath()

    def test_speed_angular(self):
        self.appendX(self.lin(duration=2, velocity=0))
        self.syncTimeline()

        tlx, tlth = self.circular_path(radius=0, phi=np.pi, duration=1.8, acc_percentage=0.35, dec_percentage=0.35)
        self.appendX(tlx)
        self.appendTH(tlth)

        self.syncTimeline()

        self.appendReversePath()

    def test_speed_circula_path(self):
        #self.appendX(self.lin(duration=3, velocity=0))
        self.syncTimeline()

        tlx, tlth = self.circular_path(radius=3, phi=np.pi/2, duration=10, acc_percentage=0.2, dec_percentage=0.2)
        self.appendX(tlx)
        self.appendTH(tlth)

        self.syncTimeline()

        #self.appendReversePath()

    def tmp_pose(self):

        jtp_flow_data = [list(), list()]

        jtp_flow_data[1].append(JTP(rel_time=2.5, **ArmMovement.pose_folded_grip_right_c4))

        self.appendArms(jtp_flow_data, True)
        self.syncTimeline()

    def mimic(self):

        self.appendMimic('ask')
        self.appendX(self.lin(velocity=0, duration=5))
        self.syncTimeline()

        self.appendMimic()
        self.appendX(self.lin(velocity=0, duration=0.1))
        self.syncTimeline()

    def led(self):

        freq = 3

        start_color = [0, 1, 0.7]
        end_color = [0.7, 0, 0]
        colorsteps = 15
        interp = list()

        for cch in range(3):
            interp.append(np.linspace(start_color[cch], end_color[cch], colorsteps))

        for r, g, b in zip(*interp):
            self.appendLed(r=r, g=g, b=b, frequency=freq, mode=3)
            self.appendX(self.lin(velocity=0, duration=1.0/freq))
            print 1.0/freq
            self.syncTimeline()

    def gripper(self):
        self.appendGripperLeft([JTP(0, **GripperMovement.gripper_pose_open)])
        self.appendX(self.lin(velocity=0, duration=6))
        self.syncTimeline()

        self.appendGripperLeft([JTP(0, **GripperMovement.gripper_pose_home)])
        self.appendX(self.lin(velocity=0, duration=1))
        self.syncTimeline()


class BoringScene_1_2_3(BaseScene):
    def __init__(self, profile):
        super(BoringScene_1_2_3, self).__init__(profile)

    def bridge_act_1_arms_startpos(self, dotime=8):
        self.appendArms(self.movePose(duration=dotime, pose=self.inject_zero_velocity(ArmMovement.pose_boring_walk_front_back_c1)))
        self.appendMimic('tired')
        self.appendLed()
        self.syncTimeline()

    def lab_act_1_slender_around(self):
        self.act_1_slender_around(steps=1, linspeed=0.3, dotime_factor=1)

    def act_1_slender_around(self, steps=20, linspeed=0.3, dotime_factor=2):
        dotime = 3.5 * (steps+1) * dotime_factor
        stepduration = dotime / (steps + 1.0)

        sin = self.sin(0, np.pi/2, stepduration/4.0)
        sinrev = sin[::-1]
        cos = self.cos(0, np.pi*2, stepduration)

        thspeed = 0.4
        sin *= thspeed  
        cos *= thspeed

        ioscale = 0.55

        self.appendX(self.lin(velocity=0, duration=stepduration/2.0))

        self.syncTimeline()

        self.appendX(self.acc(velocity_start=0, velocity_end=linspeed, duration=stepduration/2.0))

        self.syncTimelineBase()

        fillData = [None]*self.calc_samples(1.75)
        self.ARML_GOAL.extend(fillData)
        self.ARMR_GOAL.extend(fillData)
        self.appendArms(self.buildSlenderArms(dotime_step=1.75, times=steps*2))

        self.appendX(self.lin(velocity=linspeed, duration=stepduration/2.0))

        self.appendTH(sin*-ioscale)
        self.appendTH(sinrev*-ioscale)

        self.appendTH(sin)
        self.appendX(self.lin(velocity=linspeed, duration=stepduration))

        for i in range(steps):
            self.appendX(self.lin(velocity=linspeed, duration=stepduration))
            self.appendTH(cos)

        self.appendTH(sinrev)

        self.appendTH(sin*-ioscale)
        self.appendTH(sinrev*-ioscale)

        self.appendX(self.acc(velocity_start=linspeed, velocity_end=0, duration=stepduration/2.0))

        self.syncTimeline()

    def bridge_act_2_arms_startpos(self, dotime=8):
        self.appendArms(self.movePose(duration=dotime,
                                      pose=self.inject_zero_velocity(ArmMovement.pose_boring_walk_front_back_c1)))
        self.appendMimic('bored')
        self.appendLed()
        self.syncTimeline()

    def lab_act_2_1_to_window(self):
        self.act_2_1_to_window(radius=-1.5, radius_dotime=12)

    def act_2_1_to_window(self, acc_time=1, lin_time=2, radius=-2.5, radius_dotime=20, armtime=2):
        self.syncTimeline()

        self.new_section('annaehern')

        tlx, tlth = self.circular_path(radius=radius, phi=-np.pi*7/16, duration=radius_dotime, acc_percentage=0,
                                       dec_percentage=0.5)

        self.appendX(self.acc(velocity_start=0, velocity_end=tlx[0], duration=acc_time))
        self.appendX(self.lin(tlx[0], lin_time))

        self.new_section('ausrichtung fenster')
        self.appendX(tlx)
        self.appendTH(tlth)

        fillData = [None]*self.calc_samples(radius_dotime)
        self.ARML_GOAL.extend(fillData)
        self.ARMR_GOAL.extend(fillData)
        left_data, right_data = self.movePose(duration=armtime, pose=ArmMovement.pose_waiting_arms_side)
        self.ARML_GOAL.append(left_data)
        self.ARMR_GOAL.append(right_data)

        self.appendX(self.lin(duration=0.25, velocity=0))
        self.appendTH(self.lin(duration=0.25, velocity=0))

        tlx, tlth = self.circular_path(radius=0, phi=-np.pi/4, duration=3.5, acc_percentage=0.6, dec_percentage=0.4)
        self.appendX(tlx)
        self.appendTH(tlth)
        self.syncTimelineBase()

        self.appendX(self.lin(duration=1, velocity=0))
        self.syncTimelineBase()

        tlx, tlth = self.circular_path(radius=0, phi=np.pi/2, duration=6, acc_percentage=0.4, dec_percentage=0.5)
        self.appendX(tlx)
        self.appendTH(tlth)
        self.syncTimelineBase()

        self.appendX(self.lin(duration=1, velocity=0))
        self.syncTimeline()

    def lab_act_2_2_away_from_window(self):
        self.act_2_2_away_from_window(arm_dotime=2, lin_time=0)

    def act_2_2_away_from_window(self, arm_dotime=2, lin_time=6):
        self.new_section('\nenfernen vom fenster')

        self.appendArms(self.movePose(duration=arm_dotime, pose=ArmMovement.pose_boring_walk_front_back_c1))

        self.appendY(self.lin(velocity=0, duration=0.5))
        self.appendY(self.lin_acc(velocity_start=0, velocity_lin=0.15, velocity_end=0, acc_percentage=0.3, dec_percentage=0.3, duration=3.5))

        self.appendTH(self.lin_acc(velocity_start=0, velocity_lin=0.55, velocity_end=0, acc_percentage=0.35, dec_percentage=0.25, duration=5))

        self.appendX(self.lin_acc(velocity_start=0, velocity_lin=-0.08, velocity_end=0, acc_percentage=0.7, dec_percentage=0.3, duration=1.5))

        self.appendX(self.lin_acc(velocity_start=0, velocity_lin=0.2, velocity_end=0, acc_percentage=0.15, dec_percentage=0.2, duration=5 + lin_time))

        self.syncTimeline()

    def bridge_act_3_arms_startpos(self, dotime=8):
        self.appendArms(self.movePose(duration=dotime, pose=self.inject_zero_velocity(ArmMovement.pose_boring_walk_front_back_c1)))
        self.appendMimic('tired')
        self.appendLed()
        self.syncTimeline()

    def act_3_move_corner_shock(self, radius=-1.5, duration=9.0, shocktime=1.75, premimic_time=4.5, lin_time=6):
        self.syncTimeline()

        tlx, tlth = self.circular_path(radius=radius, phi=-np.radians(90),
                                       duration=duration, dec_percentage=0.5 / duration)
        self.appendX(tlx)
        self.appendTH(tlth)

        self.MIMIC.extend([None]*self.calc_samples(duration-premimic_time))
        self.appendMimic('surprised', repeat=5)

        self.syncTimeline()

        self.appendLed(frequency=1)

        tlx, tlth = self.circular_path(radius=0, phi=-np.radians(70), duration=1.5,
                                       acc_percentage=0.5, dec_percentage=0.5)
        self.appendX(tlx)
        self.appendTH(tlth)


        jtp_flow_data = [list(), list()]
        jtp_flow_data[0].append(JTP(rel_time=shocktime, **self.inject_zero_velocity(ArmMovement.pose_shock_front_left)))
        jtp_flow_data[1].append(JTP(rel_time=shocktime, **self.inject_zero_velocity(ArmMovement.pose_shock_front_right)))
        self.appendArms(jtp_flow_data, True)

        self.syncTimeline()

        self.appendY(self.lin(velocity=0, duration=0.5))
        self.appendY(self.lin_acc(velocity_start=0, velocity_lin=0.15, velocity_end=0, acc_percentage=0.3, dec_percentage=0.3, duration=3.5))

        self.appendTH(self.lin_acc(velocity_start=0, velocity_lin=0.55, velocity_end=0, acc_percentage=0.35, dec_percentage=0.25, duration=5))

        self.appendX(self.lin_acc(velocity_start=0, velocity_lin=-0.08, velocity_end=0, acc_percentage=0.7, dec_percentage=0.3, duration=1.5))

        self.appendX(self.lin_acc(velocity_start=0, velocity_lin=0.2, velocity_end=0, acc_percentage=0.15, dec_percentage=0.2, duration=5 + lin_time))

        self.syncTimeline()

class RunAwayScene_4(BaseScene):
    def __init__(self, profile):
        super(RunAwayScene_4, self).__init__(profile)

    def bridge_act_4_arms_startpos(self, dotime=8):
        self.appendArms(self.movePose(duration=dotime, pose=self.inject_zero_velocity(ArmMovement.pose_run_arms)))
        self.appendMimic('search', speed=1.0/1.5)
        self.appendLed(frequency=0.25/1.5)
        self.syncTimeline()

    def act_4_run_away(self, dotime=15, enable_base=False):
        self.syncTimeline()
        self.appendArms(self.movePose(duration=1, pose=ArmMovement.pose_run_arms))

        self.syncTimeline()

        if enable_base:
            self.appendX(self.lin_acc(velocity_start=0, velocity_lin=0.6,
                                      velocity_end=0, duration=dotime,
                                      acc_percentage=0.2, dec_percentage=0.2))

        dotime_step = 1.25
        ntimes = int(np.ceil(dotime / dotime_step))

        sin = self.sin(0, np.pi/2, dotime_step/4.0)
        sinrev = sin[::-1]
        cos = self.cos(0, np.pi*2, dotime_step)

        j1speed = 0.5
        j4speed = 0.5
        j5speed = 0.5
        j6speed = 0.5

        self.appendVelArmLeft(j1=sin*-j1speed, j4=sin*j4speed, j5=sin*j5speed, j6=sin*j6speed)
        self.appendVelArmRight(j1=sin*-j1speed, j4=sin*j4speed, j5=sin*j5speed, j6=sin*j6speed)

        for i in range(ntimes):
            self.appendVelArmLeft(j1=cos*-j1speed, j4=cos*j4speed, j5=cos*j5speed, j6=cos*j6speed)
            self.appendVelArmRight(j1=cos*-j1speed, j4=cos*j4speed, j5=cos*j5speed, j6=cos*j6speed)

        self.appendVelArmLeft(j1=sinrev*-j1speed, j4=sinrev*j4speed, j5=sinrev*j5speed, j6=sinrev*j6speed)
        self.appendVelArmRight(j1=sinrev*-j1speed, j4=sinrev*j4speed, j5=sinrev*j5speed, j6=sinrev*j6speed)

        self.syncTimeline()
        self.appendSwitchVelToGoalTimeout()
        self.syncTimeline()

        self.appendArms(self.movePose(duration=1, pose=ArmMovement.pose_run_arms))


class FindingRoseScene_5(BaseScene):
    def __init__(self, profile):
        super(FindingRoseScene_5, self).__init__(profile)

    def bridge_act_5_arm_right_startpos(self, dotime=8):
        jtp_flow_data = [list(), list()]

        save_c1 = dict(ArmMovement.pose_grip_rose_right)
        save_c1['p2'] = 0

        jtp_flow_data[0].append(JTP(rel_time=dotime, **self.inject_zero_velocity(ArmMovement.pose_right_folded_back)))
        jtp_flow_data[1].append(JTP(rel_time=dotime, **save_c1))
        jtp_flow_data[1].append(JTP(rel_time=dotime, **ArmMovement.pose_home))
        jtp_flow_data[1].append(JTP(rel_time=dotime, **self.inject_zero_velocity(ArmMovement.pose_right_folded_back)))
        self.appendArms(jtp_flow_data, True)

        self.appendMimic('happy')
        self.appendLed(frequency=1)

        self.syncTimeline()

    def calibrate_act_5_pose_grip_rose_right(self, dotime=8):
        jtp_flow_data = [list(), list()]
        jtp_flow_data[1].append(JTP(rel_time=dotime, **self.inject_zero_velocity(ArmMovement.pose_grip_rose_right)))
        self.appendArms(jtp_flow_data, True)

    def act_5_1_griper_to_rose(self):

        jtp_flow_data = [list(), list()]

        jtp_flow_data[1].append(JTP(rel_time=1.75, **ArmMovement.pose_folded_grip_right_c1))
        jtp_flow_data[1].append(JTP(rel_time=2.75, **ArmMovement.pose_folded_grip_right_c2))
        jtp_flow_data[1].append(JTP(rel_time=1.25, **ArmMovement.pose_folded_grip_right_c3))
        jtp_flow_data[1].append(JTP(rel_time=2.5, **ArmMovement.pose_folded_grip_right_c4))

        self.GRIPPERR_GOAL.extend([None]*self.calc_samples(3))
        self.appendGripperRight([JTP(0, **GripperMovement.gripper_pose_open)])

        jtp_flow_data[1].append(JTP(rel_time=2, **self.inject_zero_velocity(ArmMovement.pose_grip_rose_right)))

        self.appendArms(jtp_flow_data, True)
        self.syncTimeline()


    def act_5_2_grip_rose(self):
        self.syncTimeline()

        self.appendGripperRight([JTP(0, **GripperMovement.gripper_pose_close)])
        self.appendX(self.lin(velocity=0, duration=1))
        self.syncTimeline()

        self.syncTimeline()

    def act_5_3_gripper_away_from_rose(self):
        jtp_flow_data = [list(), list()]

        jtp_flow_data[1].append(JTP(rel_time=2, **ArmMovement.pose_folded_grip_right_c6))
        jtp_flow_data[1].append(JTP(rel_time=2, **ArmMovement.pose_folded_grip_right_c4))

        pargs = dict(ArmMovement.pose_carry_rose_front_right)
        pargs.update({'v1': 0, 'v2': 0, 'v3': 0, 'v4': 0, 'v5': 0, 'v6': 0, 'v7': 0})
        jtp_flow_data[1].append(JTP(rel_time=2, **pargs))

        self.appendArms(jtp_flow_data, True)

    def act_5_4_drive_away(self):

        sleeptime = 1.5
        self.appendX(self.lin(velocity=0, duration=sleeptime))
        self.appendY(self.lin(velocity=0, duration=sleeptime))
        self.appendTH(self.lin(velocity=0, duration=sleeptime))


        self.appendY(self.lin(velocity=0, duration=0.5))
        self.appendY(self.lin_acc(velocity_start=0, velocity_lin=0.15, velocity_end=0, acc_percentage=0.3, dec_percentage=0.3, duration=3.5))

        self.appendTH(self.lin_acc(velocity_start=0, velocity_lin=0.7, velocity_end=0, acc_percentage=0.35, dec_percentage=0.25, duration=5))

        self.appendX(self.lin_acc(velocity_start=0, velocity_lin=-0.08, velocity_end=0, acc_percentage=0.7, dec_percentage=0.3, duration=1.5))

        self.appendX(self.lin_acc(velocity_start=0, velocity_lin=0.2, velocity_end=0, acc_percentage=0.15, dec_percentage=0.2, duration=10.5))


        self.syncTimeline()


class ThePresentScene_6(BaseScene):
    def __init__(self, profile):
        super(ThePresentScene_6, self).__init__(profile)

    def bridge_act_6_arm_right_startpos(self, dotime=8):
        jtp_flow_data = [list(), list()]
        jtp_flow_data[0].append(JTP(rel_time=dotime, **self.inject_zero_velocity(ArmMovement.pose_boring_walk_front_back_c1)))
        jtp_flow_data[1].append(JTP(rel_time=dotime, **self.inject_zero_velocity(ArmMovement.pose_carry_rose_front_right)))
        self.appendArms(jtp_flow_data, True)

        self.appendMimic('sad')
        self.appendLed(frequency=1)

        self.syncTimeline()

    def act_6_give_rose(self, velocity=0.4, lin_duration=4, acc_duration=1.5, wait_duration=1, kiss_duration=3):
        self.appendX(self.acc(velocity_start=0, velocity_end=velocity, duration=acc_duration))
        self.appendX(self.lin(velocity=velocity, duration=lin_duration))

        self.syncTimeline()

        jtp_flow_data = [list(), list()]
        jtp_flow_data[1].append(JTP(rel_time=acc_duration*3, **ArmMovement.pose_present_give_rose_right))
        self.appendArms(jtp_flow_data, True)

        self.appendX(self.acc(velocity_start=velocity, velocity_end=0, duration=acc_duration*3))

        self.syncTimeline()

        self.appendGripperRight([JTP(0, **GripperMovement.gripper_pose_open)])
        self.appendGripperRight([JTP(0, **GripperMovement.gripper_pose_close)])
        # waiting...
        self.appendX(self.lin(velocity=0, duration=wait_duration))

        self.syncTimeline()



        jtp_flow_data = [list(), list()]
        jtp_flow_data[1].append(JTP.get_mirrored_jtp(JTP(rel_time=acc_duration*3, **self.inject_zero_velocity(ArmMovement.pose_boring_walk_front_back_c1))))
        self.appendArms(jtp_flow_data, True)

        self.appendGripperRight([JTP(0, **GripperMovement.gripper_pose_close)])
        self.appendX(self.lin(velocity=0, duration=1))

        self.appendMimic('laugh', repeat=10)

        #go back...

        freq = 1.5
        start_color = [0, 1, 0.7]
        end_color = [1, 0, 0]
        colorsteps = 5
        interp = list()

        for cch in range(3):
            interp.append(np.linspace(start_color[cch], end_color[cch], colorsteps))

        for r, g, b in zip(*interp):
            self.appendLed(r=r, g=g, b=b, frequency=freq, mode=3)
            fillWait = [None] * self.calc_samples(1.0/freq)
            self.LED.extend(fillWait)

        self.appendX(self.lin(velocity=0, duration=kiss_duration))
        self.syncTimeline()

        self.appendX(self.acc(velocity_start=0, velocity_end=-velocity, duration=acc_duration*3))
        self.appendX(self.lin(velocity=-velocity, duration=lin_duration))
        self.appendX(self.acc(velocity_start=-velocity, velocity_end=0, duration=acc_duration))


class CheeringScene_7_8_9_10(BaseScene):
    def __init__(self, profile):
        super(CheeringScene_7_8_9_10, self).__init__(profile)

    def bridge_act_7_arms_startpos(self, dotime=8):
        self.appendArms(self.movePose(duration=dotime, pose=self.inject_zero_velocity(ArmMovement.pose_cheer_arms_up)))
        self.appendMimic('laugh')
        self.appendLed()
        self.syncTimeline()

    def act_7_cheer_arms_up(self, lin_speed=0.5, dotime=1, ntimes=8, enable_base=False):

        self.syncTimeline()

        lintime = dotime * 2 * ntimes
        if enable_base:
            self.appendX(self.lin_acc(velocity_start=0, velocity_lin=lin_speed, velocity_end=0, duration=lintime))

        zero = self.lin(0, dotime)
        cos_0_pi = self.cos(0, np.pi, dotime)
        cos_pi_2pi = self.cos(np.pi, np.pi*2, dotime)
        sin_0_2pi = self.sin(0, np.pi*2, dotime)

        jj = [
                [ # j1
                    [sin_0_2pi, sin_0_2pi],  # j1 signal
                    [-0.3, 0.3],  # j1 left and right speed
                ],
                [ # j2
                    [zero, zero],
                    [0, 0],  # j2 left and right speed
                ],
                [ # j3
                    [cos_0_pi, cos_pi_2pi],
                    [0.4, 0.4],
                ],
                [ # j4
                    [sin_0_2pi, sin_0_2pi],
                    #[zero, zero],
                    [-0.4, 0.4],
                ],
                [ # j5
                    [zero, zero],
                    [0, 0],
                ],
                [ # j6
                    [zero, zero],
                    [0, 0],
                ],
                [ # j7
                    [zero, zero],
                    [0, 0],
                ],
        ]

        jointlist = list()
        for jn in range(7):
            step_data, velocities = zip(*zip(*jj[jn]))
            jointlist.append(map(lambda step: [step, velocities[0], velocities[1]], step_data))

        steplist = zip(*jointlist)


        for _ in range(ntimes):
            for step in steplist:
                joint_names = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'j7']
                joint_data, left_velocity, right_velocity = zip(*step)
                joint_data_left = map(op.mul, joint_data, left_velocity)
                joint_data_right = map(op.mul, joint_data, right_velocity)

                print dict(zip(joint_names, joint_data_left))

                self.appendVelArmLeft(**dict(zip(joint_names, joint_data_left)))
                self.appendVelArmRight(**dict(zip(joint_names, joint_data_right)))

        self.syncTimeline()

        self.appendVelArm(j1=np.array([0], np.float64))

        self.syncTimeline()

        self.appendSwitchVelToGoalTimeout()

        self.syncTimeline()

        self.appendArms(self.movePose(duration=4, pose=ArmMovement.pose_cheer_arms_up))

        self.syncTimeline()

    def bridge_act_8_arms_startpos(self, dotime=8):
        self.appendArms(self.movePose(duration=dotime, pose=self.inject_zero_velocity(ArmMovement.pose_cheer_arms_up)))
        self.appendMimic('laugh')
        self.syncTimeline()

    def act_8_cheering_turn(self, lin_speed=0.55, acctime=1.5, phi=np.pi*2, rot_duration=8, arm_duration=2.75):
        arm_sleep = rot_duration - arm_duration * 2
        assert arm_sleep >= 0

        arm_sleep_samples = int(self.calc_samples(arm_sleep))

        self.syncTimeline()

        self.appendX(self.acc(velocity_start=0, velocity_end=lin_speed, duration=acctime))

        self.syncTimeline()

        self.appendArms(self.movePose(duration=arm_duration, pose=ArmMovement.pose_cheer_turn))

        tlx, tly, tlth = self.const_direction_rotation(velocity_start_x=lin_speed, velocity_start_y=0, phi=phi, duration=rot_duration)
        self.appendX(tlx)
        self.appendY(tly)
        self.appendTH(tlth)

        fill_data = [None]*arm_sleep_samples
        self.ARML_GOAL.extend(fill_data)
        self.ARMR_GOAL.extend(fill_data)

        self.appendArms(self.movePose(duration=arm_duration, pose=ArmMovement.pose_cheer_arms_up))


        self.appendX(self.acc(velocity_start=lin_speed, velocity_end=0, duration=acctime))

        self.syncTimeline()

    def bridge_act_9_arms_startpos(self, dotime=8):
        self.appendArms(self.movePose(duration=dotime, pose=self.inject_zero_velocity(ArmMovement.pose_run_arms)))
        self.appendMimic('laugh')

    def act_9_drumming_rotmove_side_drive(self, lin_speed=0.25, acc_duration=1, rot_duration=2, ndrums=3,
                                          pre_lin_duration=3, post_lin_duration=3):
        # SETUP
        ########

        phi = np.pi / 2
        goto_drum_pose_duration = 4.5

        pre_ld1 = pre_lin_duration - (goto_drum_pose_duration - rot_duration)
        pre_ld2 = goto_drum_pose_duration - rot_duration

        assert pre_ld1 >= 0
        assert pre_ld2 >= 0
        assert goto_drum_pose_duration <= (rot_duration + pre_lin_duration)

        dotime_step = 1.25

        lin_duration = ndrums * dotime_step + dotime_step / 2

        sin = self.sin(0, np.pi/2, dotime_step/4.0)
        sinrev = sin[::-1]
        cos = self.cos(0, np.pi*2, dotime_step)

        j1speed = 0.5
        j4speed = 0.5
        j5speed = 0.5
        j6speed = 0.5


        # MOVEMENT
        ###########

        self.syncTimeline()

        self.appendX(self.acc(velocity_start=0, velocity_end=lin_speed, duration=acc_duration))


        self.syncTimeline()

        self.appendX(self.lin(lin_speed, pre_ld1))

        self.syncTimeline()

        self.appendX(self.lin(lin_speed, pre_ld2))
        self.syncTimelineBase()

        pargs = dict(ArmMovement.pose_cheer_drum)
        pargs.update({'v1': 0, 'v2': 0, 'v3': 0, 'v4': 0, 'v5': 0, 'v6': 0, 'v7': 0})
        self.appendArms(self.movePose(duration=goto_drum_pose_duration, pose=pargs))

        tlx, tly, tlth = self.const_direction_rotation(velocity_start_x=lin_speed, velocity_start_y=0, phi=phi, duration=rot_duration)
        self.appendX(tlx)
        self.appendY(tly)
        self.appendTH(tlth)

        #self.syncTimeline()

        self.syncTimelineArmVelocity()

        self.appendY(self.lin(-lin_speed, lin_duration))

        self.appendVelArmLeft(j1=sin*-j1speed, j4=sin*j4speed, j5=sin*j5speed, j6=sin*j6speed)
        for _ in range(ndrums):
            self.appendVelArmLeft(j1=cos*-j1speed, j4=cos*j4speed, j5=cos*j5speed, j6=cos*j6speed)
        self.appendVelArmLeft(j1=sinrev*-j1speed, j4=sinrev*j4speed, j5=sinrev*j5speed, j6=sinrev*j6speed)

        #sin *= -1
        #sinrev *= -1
        #cos *= -1
        self.appendVelArmRight(j1=sin*-j1speed, j4=sin*j4speed, j5=sin*j5speed, j6=sin*j6speed)
        for _ in range(ndrums):
            self.appendVelArmRight(j1=cos*-j1speed, j4=cos*j4speed, j5=cos*j5speed, j6=cos*j6speed)
        self.appendVelArmRight(j1=sinrev*-j1speed, j4=sinrev*j4speed, j5=sinrev*j5speed, j6=sinrev*j6speed)



        self.syncTimeline()

        self.appendSwitchVelToGoalTimeout()
        self.syncTimelineArmGoal()

        tlx, tly, tlth = self.const_direction_rotation(velocity_start_x=0, velocity_start_y=-lin_speed, phi=-phi, duration=rot_duration)
        self.appendX(tlx)
        self.appendY(tly)
        self.appendTH(tlth)

        self.appendArms(self.movePose(duration=goto_drum_pose_duration, pose=ArmMovement.pose_run_arms))

        self.appendX(self.lin(lin_speed, post_lin_duration))

        self.appendX(self.acc(velocity_start=lin_speed, velocity_end=0, duration=acc_duration))

    def bridge_10_mimic(self, dotime=8):
        self.appendMimic('laugh')

        jtp_flow_data = [list(), list()]
        jtp_flow_data[0].append(JTP(rel_time=dotime, **ArmMovement.pose_right_folded_back))
        jtp_flow_data[1].append(JTP(rel_time=dotime, **ArmMovement.pose_right_folded_back))
        self.appendArms(jtp_flow_data, True)

        self.syncTimeline()

    def act_10_corner_rotation(self, duration=16, scalex=1, scaley=1):
        points = [[0, 0], [4, 0], [1, 3]]

        points = np.array(points, np.float)
        xpoints, ypoints = zip(*points)
        xpoints = np.array(xpoints, np.float)
        ypoints = np.array(ypoints, np.float)
        xpoints *= scalex
        ypoints *= scaley
        points = zip(xpoints, ypoints)

        x, th = self.createBezier(points, duration=duration)

        self.appendX(self.acc(velocity_start=0, velocity_end=x[0], duration=1))
        self.syncTimeline()

        _, tlth = self.circular_path(radius=0, phi=-np.pi*2, duration=duration, acc_percentage=0.2, dec_percentage=0.2)

        absphi = integrate.cumtrapz(tlth*self.profile.sample_time)
        thover = th + tlth

        absphi *= np.pi*2/np.abs(absphi).max()

        xvel = x * np.cos(absphi)
        yvel = x * -np.sin(absphi)

        self.appendX(xvel)
        self.appendY(yvel)
        self.appendTH(thover)

        self.appendX(self.lin(velocity=x[-1], duration=5))

        self.appendX(self.acc(velocity_start=x[-1], velocity_end=0, duration=1))


class EndingScene_11(BaseScene):
    def __init__(self, profile):
        super(EndingScene_11, self).__init__(profile)

    def bridge_act_11_mimic(self, dotime=8):
        self.appendMimic('blink_right')

        jtp_flow_data = [list(), list()]
        jtp_flow_data[0].append(JTP(rel_time=dotime, **ArmMovement.pose_right_folded_back))
        jtp_flow_data[1].append(JTP(rel_time=dotime, **ArmMovement.pose_right_folded_back))
        self.appendArms(jtp_flow_data, True)

        self.syncTimeline()

    def act_11_the_end(self, speed=0.5, lin_duration=2, acc_duration=1.5):
        self.appendX(self.acc(velocity_start=0, velocity_end=speed, duration=acc_duration))
        self.appendX(self.lin(velocity=speed, duration=lin_duration))
        self.appendX(self.acc(velocity_start=speed, velocity_end=0, duration=acc_duration))


if __name__ == '__main__':
    cob4_2_profile = Profile(rate=70, max_linear_velocity=0.7, max_angular_velocity=2.7,
                             max_linear_acceleration=0.022, max_angular_acceleration=0.074, switch_vel_to_goal_timeout=0.7  )

    dummy = DummyScene(profile=cob4_2_profile)
    test = StuffToTest(profile=cob4_2_profile)

    boring = BoringScene_1_2_3(profile=cob4_2_profile)
    discover = RunAwayScene_4(profile=cob4_2_profile)
    findrose = FindingRoseScene_5(profile=cob4_2_profile)
    present = ThePresentScene_6(profile=cob4_2_profile)
    cheer = CheeringScene_7_8_9_10(profile=cob4_2_profile)
    ending = EndingScene_11(profile=cob4_2_profile)


    #boring.bridge_act_1_arms_startpos()
    #boring.lab_act_1_slender_around()
    #boring.act_1_slender_around()
    #boring.bridge_act_2_arms_startpos()
    #boring.lab_act_2_1_to_window()
    boring.act_2_1_to_window()
    #boring.lab_act_2_2_away_from_window()
    #boring.act_2_2_away_from_window()

    #boring.act_3_move_corner_shock()

    #discover.lab_act_4_run_away()
    #discover.act_4_run_away()

    #findrose.act_5_1_griper_to_rose()
    #findrose.act_5_2_grip_rose()
    #findrose.act_5_3_gripper_away_from_rose()
    #findrose.act_5_4_drive_away()

    #present.act_6_give_rose()

    #cheer.bridge_act_7_arms_startpos()
    #cheer.lab_act_7_cheer_arms_up()
    #cheer.act_7_cheer_arms_up()

    #cheer.lab_act_8_cheering_turn()
    cheer.act_8_cheering_turn()

    #cheer.lab_act_9_drumming_rotmove_side_drive()
    #cheer.act_9_drumming_rotmove_side_drive()

    #cheer.lab_act_10_corner_rotation()
    #cheer.act_10_corner_rotation()

    #ending.act_11_the_end()

    #test.test_map()
    test.test_speed_linear()
    #test.test_speed_angular()
    #test.test_speed_circula_path()

    #test.testBezier()
    #test.gripper()

    #boring.appendReversePath()
    #test.appendReversePath()

    #present.act_6_give_rose()

    # SETTING MASTER TIMELINE
    ##########################

    #masterTimeline = boring
    #masterTimeline = discover
    #masterTimeline = findrose
    #masterTimeline = present
    masterTimeline = cheer
    #masterTimeline = test
    #masterTimeline = ending

    sh = ServiceHandler()
    sh.add_service_callback('scenario/br1', boring.bridge_act_1_arms_startpos, boring)
    sh.add_service_callback('scenario/sc1', boring.act_1_slender_around, boring)
    sh.add_service_callback('scenario/br2', boring.bridge_act_2_arms_startpos, boring)
    sh.add_service_callback('scenario/sc2', [boring.act_2_1_to_window, boring.act_2_2_away_from_window], boring)
    sh.add_service_callback('scenario/br3', boring.bridge_act_3_arms_startpos, boring)
    sh.add_service_callback('scenario/sc3', boring.act_3_move_corner_shock, boring)
    sh.add_service_callback('scenario/br4', discover.bridge_act_4_arms_startpos, discover)
    sh.add_service_callback('scenario/sc4', discover.act_4_run_away, discover)

    sh.add_service_callback('scenario/cal5', findrose.calibrate_act_5_pose_grip_rose_right, findrose)
    sh.add_service_callback('scenario/br5', findrose.bridge_act_5_arm_right_startpos, findrose)
    sh.add_service_callback('scenario/sc5', [findrose.act_5_1_griper_to_rose, findrose.act_5_2_grip_rose,
                                             findrose.act_5_3_gripper_away_from_rose, findrose.act_5_4_drive_away], findrose)
    sh.add_service_callback('scenario/br6', present.bridge_act_6_arm_right_startpos, present)
    sh.add_service_callback('scenario/sc6', present.act_6_give_rose, present)

    sh.add_service_callback('scenario/br7', cheer.bridge_act_7_arms_startpos, cheer)
    sh.add_service_callback('scenario/sc7', cheer.act_7_cheer_arms_up, cheer)
    sh.add_service_callback('scenario/br8', cheer.bridge_act_8_arms_startpos, cheer)
    sh.add_service_callback('scenario/sc8', cheer.act_8_cheering_turn, cheer)
    sh.add_service_callback('scenario/br9', cheer.bridge_act_9_arms_startpos, cheer)
    sh.add_service_callback('scenario/sc9', cheer.act_9_drumming_rotmove_side_drive, cheer)
    sh.add_service_callback('scenario/br10', cheer.bridge_10_mimic, cheer)
    sh.add_service_callback('scenario/sc10', cheer.act_10_corner_rotation, cheer)

    sh.add_service_callback('scenario/br11', ending.bridge_act_11_mimic, ending)
    sh.add_service_callback('scenario/sc11', ending.act_11_the_end, ending)

    sh.add_service_callback('scenario/test1', test.mimic, test)
    sh.add_service_callback('scenario/test2', test.led, test)
    sh.add_service_callback('scenario/test3', test.gripper, test)

    sh.startup(masterTimeline)
