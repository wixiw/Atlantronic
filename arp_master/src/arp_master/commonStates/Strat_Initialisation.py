#!/usr/bin/env python


#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
import os
from Waiting import *
from MotorManagement import *

from arp_master.fsmFramework import *

class InitTurretZeros(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['succeeded','problem'])
        with self:
            smach.StateMachine.add('SetSteeringPower',
                      SetSteeringPower(),
                      transitions={'succeeded':'FindSteeringZero', 'timeout':'problem'})
            
            smach.StateMachine.add('FindSteeringZero',
                      FindSteeringZero(),
                      transitions={'succeeded':'BackToPositionTurretMode', 'timeout':'problem'})
            
            smach.StateMachine.add('BackToPositionTurretMode',
                      SetSteeringMotorModeState("position"),
                      transitions={'succeeded':'SetDrivingPower', 'timeout':'problem'})
            
            smach.StateMachine.add('SetDrivingPower',
                      SetDrivingPower(),
                      transitions={'succeeded':'succeeded', 'timeout':'problem'})


        
#the first state: only to wait for the start to be unpluged 
class Init(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['initstateok'])

    def executeTransitions(self):
       # the only condition verified to go on is that the start is not put
       if Inputs.getstart()==1:
           return 'initstateok'

#the state that will wait orocos software to deploy
class WaitForOrocos(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['deployed'])
    
    def executeTransitions(self):
        if Inputs.getdeployed() is True:
            return 'deployed'
    
    def executeOut(self):
        rospy.loginfo("Orocos deployed.")


            