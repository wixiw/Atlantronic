#!/usr/bin/env python


#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
import os


class StartSequence(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['gogogo','problem'])
        with self:
            smach.StateMachine.add('SetSteeringPower',
                      SetSteeringPower(),
                      transitions={'succeeded':'FindSteeringZero','timeout':'problem'})
            
            smach.StateMachine.add('FindSteeringZero',
                      FindSteeringZero(),
                      transitions={'succeeded':'BackToPositionTurretMode','timeout':'problem'})
            
            smach.StateMachine.add('BackToPositionTurretMode',
                      SetSteeringMotorModeState("position"),
                      transitions={'succeeded':'SetDrivingPower','timeout':'problem'})
            
            smach.StateMachine.add('SetDrivingPower',
                      SetDrivingPower(),
                      transitions={'succeeded':'SetInitialPosition','timeout':'problem'})
            
            smach.StateMachine.add('SetInitialPosition',
                      SetInitialPosition(1.200,0.750,-pi),
                      transitions={'succeeded':'WaitPos','timeout':'problem'})
            
            smach.StateMachine.add('WaitPos',
                      WaiterState(1),
                      transitions={'timeout':'Backward'})
                        
            smach.StateMachine.add('Backward',
                      Backward(),
                      transitions={'succeeded':'WaitForMatch','timeout':'problem'})
                        
            smach.StateMachine.add('WaitForMatch', 
                      WaitForMatch(),
                      transitions={'start':'SetInitialPosition2', 'timeout':'problem'})
            
            smach.StateMachine.add('SetInitialPosition2',
                      SetInitialPosition(1.250,0.750,-pi),
                      transitions={'succeeded':'gogogo','timeout':'problem'})
    
      
          
       
class TakeStartPosition(CyclicActionState):
    def createAction(self):
       self.cap(AmbiCapRed(-pi,Data.color).angle)
      
class Backward(CyclicActionState):
    def createAction(self):
        self.backward(0.050)
