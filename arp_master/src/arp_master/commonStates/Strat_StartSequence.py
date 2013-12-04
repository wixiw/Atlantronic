#!/usr/bin/env python


#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
import os
from SetPosition import *
from MotorManagement import *
from Waiting import *

#
# This is the default a1 level state for any strategy. Except for testing purpose it should be overided in any strategy
#
##################################################



class FindSteeringZeros(smach.StateMachine):
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



class StartSequence(smach.StateMachine):
    def __init__(self,x,y,theta):
        smach.StateMachine.__init__(self,outcomes=['gogogo','problem'])
        with self:
            PreemptiveStateMachine.add('FindSteeringZeros',
                      FindSteeringZeros(), 
                      transitions={'succeeded':'SetInitialPosition', 'timeout':'problem'})
            
            smach.StateMachine.add('SetInitialPosition',
                      SetInitialPosition(x,y,theta),
                      transitions={'succeeded':'WaitForMatch', 'timeout':'problem'})
                        
            
            smach.StateMachine.add('WaitForMatch', 
                      WaitForMatch(),
                      transitions={'start':'gogogo', 'timeout':'problem'})
    
      
