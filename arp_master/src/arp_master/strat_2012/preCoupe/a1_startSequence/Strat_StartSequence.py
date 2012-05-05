#!/usr/bin/env python


#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
import os


class StartSequence(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['gogogo','problem'])
        with self:
            smach.StateMachine.add('SetInitialPosition',
                      SetInitialPosition(1.250,0.750,-pi),
                      transitions={'succeeded':'SetSteeringPower','timeout':'problem'})
                        
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
                      transitions={'succeeded':'WaitForMatch','timeout':'problem'})
            
            smach.StateMachine.add('ShowReady',
                      ShowReady(),
                      transitions={'succeeded':'TakeStartPosition','timeout':'problem'})

            smach.StateMachine.add('TakeStartPosition',
                      TakeStartPosition(),
                      transitions={'succeeded':'WaitForMatch','timeout':'problem'})
            
            smach.StateMachine.add('WaitForMatch', 
                      WaitForMatch(),
                      transitions={'start':'gogogo', 'timeout':'problem'})
    
      
          
       
#on tourne un peu avant le match pour confirmer la couleur
class ShowReady(CyclicActionState):
    def createAction(self):
        self.cap(AmbiCapRed(-pi+0.8,Data.color).angle)
        
class TakeStartPosition(CyclicActionState):
    def createAction(self):
       self.cap(AmbiCapRed(-pi,Data.color).angle)
      
        
