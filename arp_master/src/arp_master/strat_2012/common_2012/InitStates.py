#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *

from DynamixelActionState import *

class Initialisation2012(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['endInitialisation','failed'])
        with self:
            smach.StateMachine.add('Initialisation', 
                                   Strat_Initialisation.Initialisation(),
                                   transitions={'endInitialisation':'endInitialisation', 'failed':'failed'})


class StartSequence2012(smach.StateMachine):
    def __init__(self,x,y,theta):
        smach.StateMachine.__init__(self,outcomes=['gogogo','problem'])
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
                      transitions={'succeeded':'SetInitialPosition', 'timeout':'problem'})
            
            smach.StateMachine.add('SetInitialPosition',
                      SetInitialPosition(x,y,theta),
                      transitions={'succeeded':'OpenClaws', 'timeout':'problem'})

            smach.StateMachine.add('OpenClaws',
                      ClawFingerOrder(0,0,0,0),
                      transitions={'succeeded':'CloseClaws', 'timeout':'problem'})      
            
            smach.StateMachine.add('CloseClaws',
                      ClawFingerOrder(-1.8,-1.8,-1.8,-1.8),
                      transitions={'succeeded':'WaitForMatch', 'timeout':'problem'})                    
            
            smach.StateMachine.add('WaitForMatch', 
                      WaitForMatch(),
                      transitions={'start':'gogogo', 'timeout':'problem'})
