#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *

from DynamixelActionState import *
from Table2012 import *
from DebugStates import *

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
                      transitions={'succeeded':'OpenClaws', 'timeout':'SetDrivingPower'})
            
            smach.StateMachine.add('OpenClaws',
                      FingerClawState('half_open'),
                      transitions={'succeeded':'CloseClaws', 'timeout':'problem'})      
            
            smach.StateMachine.add('CloseClaws',
                      FingerClawState('close'),
                      transitions={'succeeded':'PrepareTurrets', 'timeout':'problem'})                    
            
            smach.StateMachine.add('PrepareTurrets',
                      AmbiOpenLoopOrder(0.015,0.000,0.0, duration=0.5),
                      transitions={'succeeded':'SetInitialPosition', 'timeout':'SetInitialPosition'})
            
            smach.StateMachine.add('SetInitialPosition',
                      SetInitialPosition(x,y,theta),
                      transitions={'succeeded':'WaitForMatch0', 'timeout':'problem'})



            smach.StateMachine.add('WaitForLoc', 
                      WaiterState(2.0),
                      transitions={'timeout':'ShowReady'})
            
            smach.StateMachine.add('WaitForMatch0', 
                      WaitForMatch(),
                      transitions={'start':'ShowReady', 'timeout':'WaitForMatch0'})
            
            

            smach.StateMachine.add('ShowReady',
                      AmbiOmniDirectOrder(0.500,0.75,pi/2, vmax = 0.3),
                      transitions={'succeeded':'WaitForLoc2', 'timeout':'problem'})
            
            smach.StateMachine.add('WaitForLoc2', 
                      WaiterState(2.0),
                      transitions={'timeout':'GoHome'})
            
            smach.StateMachine.add('GoHome',
                      AmbiOmniDirectOrder(1.140,0.75,-pi, vmax = 0.3),
                      transitions={'succeeded':'WaitForStart', 'timeout':'problem'})
            
            smach.StateMachine.add('WaitForStart', 
                       WaitForStart(),
                       transitions={'start':'WaitForMatch','timeout':'WaitForStart'})
            
            smach.StateMachine.add('WaitForMatch', 
                      WaitForMatch(),
                      transitions={'start':'SetInitialPosition2', 'timeout':'WaitForMatch'})
            
            smach.StateMachine.add('SetInitialPosition2',
                      SetInitialPosition(x,y,theta),
                      transitions={'succeeded':'gogogo', 'timeout':'gogogo'})


####################################################################################################################

class Uninitialisation2012(smach.StateMachine):
    #a appeler avec la position initiale a rejoindre
    def __init__(self,x,y,theta):
        X_VALUE = 0.800
        Y_VALUE = 0.700
    
        smach.StateMachine.__init__(self,outcomes=['endUninitialisation'])
        with self:
            smach.StateMachine.add('Uninitialisation',
                      Uninitialisation(),
                      transitions={'endUninitialisation':'PrintStratInfo'})
            
            smach.StateMachine.add('PrintStratInfo',
                      PrintStratInfo(),
                      transitions={'ok':'WaitForStart'})
                        
            smach.StateMachine.add('WaitForStart',
                      WaitForStart(),
                      transitions={'start':'WaitForMatch', 'timeout':'WaitForStart'})
            
            smach.StateMachine.add('WaitForMatch',
                      WaitForMatch(),
                      transitions={'start':'CloseClaws', 'timeout':'WaitForStart'})
            
            smach.StateMachine.add('CloseClaws',
                      FingerClawState('close'),
                      transitions={'succeeded':'SelectState', 'timeout':'endUninitialisation'})
                        
            smach.StateMachine.add('SelectState',
                      SelectState(),
                      transitions={'farBot':'FarBot','farTop':'FarTop','closeTop':'CloseTop','closeBot':'CloseBot', 'timeout':'endUninitialisation'})
            
            smach.StateMachine.add('FarBot',
                      AmbiOmniDirectOrder(-X_VALUE,-Y_VALUE,pi/4),
                     transitions={'succeeded':'CloseBot', 'timeout':'endUninitialisation'})

            smach.StateMachine.add('FarTop',
                      AmbiOmniDirectOrder(-X_VALUE,Y_VALUE,-pi/4),
                     transitions={'succeeded':'CloseTop', 'timeout':'endUninitialisation'})
            
            smach.StateMachine.add('CloseTop',
                      AmbiOmniDirectOrder(X_VALUE,Y_VALUE,-3*pi/4),
                     transitions={'succeeded':'GoHome', 'timeout':'endUninitialisation'})
            
            smach.StateMachine.add('CloseBot',
                      AmbiOmniDirectOrder(X_VALUE,-Y_VALUE,3*pi/4),
                     transitions={'succeeded':'CloseTop', 'timeout':'endUninitialisation'})
            
            smach.StateMachine.add('GoHome',
                      AmbiOmniDirectOrder(x+0.058,y,theta),
                     transitions={'succeeded':'endUninitialisation', 'timeout':'endUninitialisation'})

class SelectState(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['farBot','farTop','closeTop','closeBot'])

    def executeTransitions(self):
        return Table2012.getTableHalf(Inputs.getx(), Inputs.gety(),Data.color); 
    
    
####################################################################################################################    