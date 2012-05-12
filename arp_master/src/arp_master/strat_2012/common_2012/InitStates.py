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


####################################################################################################################


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
                      FingerClawState('half_open'),
                      transitions={'succeeded':'CloseClaws', 'timeout':'problem'})      
            
            smach.StateMachine.add('CloseClaws',
                      FingerClawState('close'),
                      transitions={'succeeded':'WaitForMatch', 'timeout':'problem'})                    
            
            smach.StateMachine.add('WaitForMatch', 
                      WaitForMatch(),
                      transitions={'start':'gogogo', 'timeout':'problem'})



####################################################################################################################

class Uninitialisation(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['endUninitialisation'])
        with self:
            smach.StateMachine.add('WaitForStart',
                      WaitForStart(),
                      transitions={'start':'CloseClaws', 'timeout':'WaitForStart'})
            
            smach.StateMachine.add('CloseClaws',
                      FingerClawState('close'),
                      transitions={'succeeded':'SelectState', 'timeout':'endUninitialisation'})
                        
            smach.StateMachine.add('SelectState',
                      SelectState(),
                      transitions={'farBot':'FarBot','farTop':'FarTop','closeTop':'CloseTop','closeBot':'CloseBot', 'timeout':'endUninitialisation'})
            
            smach.StateMachine.add('FarBot',
                      AmbiOmniDirectOrder(-0.750,-0.500,pi/4),
                     transitions={'succeeded':'CloseBot', 'timeout':'endUninitialisation'})

            smach.StateMachine.add('FarTop',
                      AmbiOmniDirectOrder(-0.750,0.500,-pi/4),
                     transitions={'succeeded':'CloseTop', 'timeout':'endUninitialisation'})
            
            smach.StateMachine.add('CloseTop',
                      AmbiOmniDirectOrder(0.750,0.500,-3*pi/4),
                     transitions={'succeeded':'PrepareGoHome', 'timeout':'endUninitialisation'})
            
            smach.StateMachine.add('CloseBot',
                      AmbiOmniDirectOrder(0.750,-0.500,3*pi/4),
                     transitions={'succeeded':'CloseTop', 'timeout':'endUninitialisation'})
            
            smach.StateMachine.add('PrepareGoHome',
                      AmbiOmniDirectOrder(0.900,0.750,0),
                     transitions={'succeeded':'GoHome', 'timeout':'endUninitialisation'})
            
            smach.StateMachine.add('GoHome',
                      AmbiOmniDirectOrder(1.250,0.750,0),
                     transitions={'succeeded':'endUninitialisation', 'timeout':'endUninitialisation'})

      
class SelectState(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['farBot','farTop','closeTop','closeBot'])

    def executeTransitions(self):
        if Data.color is "red":
            if Inputs.getx() < 0 and Inputs.gety() < 0:
                return 'farBot'
            if Inputs.getx() < 0 and Inputs.gety() >= 0:
                return 'farTop'
            if Inputs.getx() >= 0 and Inputs.gety() >= 0:
                return 'closeTop'     
            #if Inputs.getx() >= 0 and Inputs.gety() < 0:
            return 'closeBot'    
        else:
            if Inputs.getx() < 0 and Inputs.gety() < 0:
                return 'closeBot'
            if Inputs.getx() < 0 and Inputs.gety() >= 0:
                return 'closeTop'
            if Inputs.getx() >= 0 and Inputs.gety() >= 0:
                return 'farTop'     
            #if Inputs.getx() >= 0 and Inputs.gety() < 0:
            return 'farBot'    
    
    
####################################################################################################################    