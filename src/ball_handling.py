#!/usr/bin/python
import rospy
import smach
import smach_ros
import tf
import sys
import copy

from std_srvs.srv import Trigger

## -- Initiation
class BallHandlingInit(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])

    def execute(self, userdata):
        #wait_for_services
        if not self.wait_for_service('/scenario/po1', 5.0): #pre_ball_start
            return "failed"
        if not self.wait_for_service('/scenario/rec1', 5.0): #hold_ball
            return "failed"
        if not self.wait_for_service('/scenario/rec2', 5.0): #z
            return "failed"
        if not self.wait_for_service('/scenario/rec3', 5.0): #roll
            return "failed"
        if not self.wait_for_service('/scenario/rec4', 5.0): #cross
            return "failed"
        if not self.wait_for_service('/scenario/rec5', 5.0): #circ8
            return "failed"
        
        rospy.loginfo("All Services available!")
        return "succeeded"

    def wait_for_service(self, srv_name, timeout):
        try:
            rospy.wait_for_service(srv_name, timeout)
            return True
        except rospy.ROSException, e:
            rospy.logerr("Service %s not available within timout!" %srv_name)
            return False
        except rospy.ROSInterruptException, e:
            rospy.logerr("InterruptRequest received")
            return False

class BallHandlingExecute(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        
        rospy.loginfo("BallHandlingExecute: INIT")
        
        self.client_pre_ball = rospy.ServiceProxy('/scenario/po1', Trigger)
        self.client_hold_ball = rospy.ServiceProxy('/scenario/rec1', Trigger)
        self.client_z = rospy.ServiceProxy('/scenario/rec2', Trigger)
        self.client_roll = rospy.ServiceProxy('/scenario/rec3', Trigger)
        self.client_cross = rospy.ServiceProxy('/scenario/rec4', Trigger)
        self.client_circ8 = rospy.ServiceProxy('/scenario/rec5', Trigger)
        
        rospy.loginfo("...done")

            
    def execute(self, userdata):
        
        rospy.loginfo("BallHandlingExecute: EXECUTE")

        rospy.loginfo("Moving to PreBall")
        if not self.call_service(self.client_pre_ball):
            rospy.logerr("Movement to PreBall failed!")
            return "failed"
            
        rospy.loginfo("Moving to HoldBall")
        if not self.call_service(self.client_hold_ball):
            rospy.logerr("Movement to HoldBall failed!")
            return "failed"
        
        rospy.loginfo("Starting Movement Z")
        if not self.call_service(self.client_z):
            rospy.logerr("Movement Z failed!")
            return "failed"
        
        rospy.loginfo("Starting Movement CROSS")
        if not self.call_service(self.client_cross):
            rospy.logerr("Movement CROSS failed!")
            return "failed"
        
        rospy.loginfo("Starting Movement CIRC8")
        if not self.call_service(self.client_circ8):
            rospy.logerr("Movement CIRC8 failed!")
            return "failed"
            
        rospy.loginfo("All Movements successfull! BallHandling finished!")
        return "succeeded"
    
    def call_service(self, proxy):
        try:
            req = TriggerRequest()
            print req
            res = proxy(req)
            print res
            if res.success:
                rospy.loginfo("Service call successful: %s"%res.message)
                return True
            else:
                rospy.logerr("Service not successful: %s"%res.message)
                return False
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return False
        

    


## -- State Machine 

class SM(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['ended'])
        with self:

            smach.StateMachine.add('BALL_HANDLING_INIT',BallHandlingInit(),
                transitions={'succeeded':'BALL_HANDLING_EXECUTE',
                             'failed':'ended'})

            smach.StateMachine.add('BALL_HANDLING_EXECUTE',BallHandlingExecute(),
                transitions={'succeeded':'ended',
                             'failed':'ended'})

if __name__=='__main__':
    rospy.init_node('ball_handling')
    sm = SM()
    sis = smach_ros.IntrospectionServer('sm', sm, 'SM')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
