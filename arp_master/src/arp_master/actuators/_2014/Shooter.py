#!/usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master.strat_2014.common_2014.Robot2014 import *

from arp_master.fsmFramework import *
from arp_master.commonStates import *
from arp_master.strat_2014 import *
from arp_master.actuators import *     

#
# This state machine allows to put the cannon in its initial state.
#
class DefaultCannonState(smach.StateMachine):
    def __init__(self, p_side):
            smach.StateMachine.__init__(self, outcomes=['done','problem'])
            
            with self:
                smach.StateMachine.add('CleanupPositions',
                       AmbiCannonBiCommand(p_side, Robot2014.cannonFingerLeftYellowPos['GOINFRONT'], Robot2014.cannonStockerLeftYellowPos['LOADING']),
                       transitions={'succeeded':'CannonPositionCleaned', 'problem':'problem'})
                                
                smach.StateMachine.add('CannonPositionCleaned',
                       AmbiCannonBiCommand(p_side, Robot2014.cannonFingerLeftYellowPos['ARMED'], Robot2014.cannonStockerLeftYellowPos['LOADING']),
                       transitions={'succeeded':'done', 'problem':'problem'})
                 
#
# This state machine allows to arm, load and shoot a ball.
#
# @param String p_side : the side of the cannon that has to shoot on the yellow configuraiton
#
class AmbiShootOneBall(smach.StateMachine):
    def __init__(self, p_side):
            smach.StateMachine.__init__(self, outcomes=['shot','blocked'])
            
            with self:      
                smach.StateMachine.add('LoadBall',
                       AmbiCannonBiCommand(p_side, Robot2014.cannonFingerLeftYellowPos['ARMED'], Robot2014.cannonStockerLeftYellowPos['LOADING']),
                       transitions={'succeeded':'WaitBallInStocker', 'problem':'blocked'})
                                
                smach.StateMachine.add('WaitBallInStocker',
                       WaiterState(0.3),
                       transitions={'timeout':'ReadyToShoot'})    
                
                smach.StateMachine.add('ReadyToShoot',
                       AmbiCannonBiCommand(p_side, Robot2014.cannonFingerLeftYellowPos['ARMED'], Robot2014.cannonStockerLeftYellowPos['UNLOADING']),
                       transitions={'succeeded':'WaitBallInShooter', 'problem':'ReLoadBall'})
                
                # ==> Blocking point A
                                
                smach.StateMachine.add('WaitBallInShooter',
                       WaiterState(0.3),
                       transitions={'timeout':'ShootBall'})

                smach.StateMachine.add('ShootBall',
                       AmbiCannonBiCommand(p_side, Robot2014.cannonFingerLeftYellowPos['SHOOT'], Robot2014.cannonStockerLeftYellowPos['UNLOADING']),
                       transitions={'succeeded':'CleanupPositions', 'problem':'blocked'})
                                
                smach.StateMachine.add('CleanupPositions',
                       AmbiCannonBiCommand(p_side, Robot2014.cannonFingerLeftYellowPos['GOINFRONT'], Robot2014.cannonStockerLeftYellowPos['LOADING']),
                       transitions={'succeeded':'CannonPositionCleaned', 'problem':'ReClean'})
                           
                # ==> Blocking point B
                                                
                smach.StateMachine.add('CannonPositionCleaned',
                       AmbiCannonBiCommand(p_side, Robot2014.cannonFingerLeftYellowPos['ARMED'], Robot2014.cannonStockerLeftYellowPos['LOADING']),
                       transitions={'succeeded':'shot', 'problem':'blocked'})                 


#Blocking point A : when the ball don't fall from Stocker to Shooter
                smach.StateMachine.add('ReLoadBall',
                       AmbiCannonBiCommand(p_side, Robot2014.cannonFingerLeftYellowPos['ARMED'], Robot2014.cannonStockerLeftYellowPos['LOADING']),
                       transitions={'succeeded':'ReadyToShootNoFailure', 'problem':'blocked'})
                
                smach.StateMachine.add('ReadyToShootNoFailure',
                       AmbiCannonBiCommand(p_side, Robot2014.cannonFingerLeftYellowPos['ARMED'], Robot2014.cannonStockerLeftYellowPos['UNLOADING']),
                       transitions={'succeeded':'WaitBallInShooter', 'problem':'blocked'})

#Blocking point B : when the CannonFinger still contain a ball while returning to LOADING potition
                smach.StateMachine.add('ReClean',
                       AmbiCannonBiCommand(p_side, Robot2014.cannonFingerLeftYellowPos['ARMED'], Robot2014.cannonStockerLeftYellowPos['UNLOADING']),
                       transitions={'succeeded':'CleanupPositionsNoFailure', 'problem':'blocked'})

                smach.StateMachine.add('CleanupPositionsNoFailure',
                       AmbiCannonBiCommand(p_side, Robot2014.cannonFingerLeftYellowPos['GOINFRONT'], Robot2014.cannonStockerLeftYellowPos['LOADING']),
                       transitions={'succeeded':'CannonPositionCleaned', 'problem':'blocked'})

#
# This state allows to send a blocking command to 2 cannon dynamixels simultaneously  
#   
# Transitions are : 'succeeded','problem' (can be either stucked or timeout)
#
# @param String p_side             : the side of the cannon (Left or Right)
# @param Double p_fingerPosition   : shooting finger command in rad
# @param Double p_stockerPosition  : stocker command in rad
#
class CannonBiCommand(DynamixelGoto):
    def __init__(self, p_side, p_fingerPosition, p_stockerPosition):
        dynamixelList = [p_side + "CannonFinger", p_side+"CannonStocker"]
        posList = [p_fingerPosition,p_stockerPosition]
        DynamixelGoto.__init__(self, dynamixelList, posList)
        
        
#
# This state allows to send configuration to both dynamixels in the cannon
#
# @param String p_side               : the side of the cannon (Left or Right)
# @param Percentage p_fingerTorque   : shooting finger torque in [20;100]
# @param Percentage p_stockerTorque  : stocker torque in [20;100]
#
class CannonBiTorqueConfig(smach.StateMachine):
    def __init__(self, p_side, p_fingerTorque, p_stockerTorque):
        smach.StateMachine.__init__(self,outcomes=['done'])
        
        with self: 
            smach.StateMachine.add('ConfigFinger',
                        DynamixelTorqueConfig(p_side + "CannonFinger", p_fingerTorque),
                        transitions={ 'done' : 'ConfigStocker' })
            
            smach.StateMachine.add('ConfigStocker',
                        DynamixelTorqueConfig(p_side + "CannonStocker", p_stockerTorque),
                        transitions={ 'done' : 'done' })

#
# This state allows to send a blocking command to 2 cannon dynamixels simultaneously depending on the match color 
#   
# see CannonBiCommand
#
class AmbiCannonBiCommand(AmbiDynamixelGoto):
    def __init__(self, p_side, p_fingerPosition, p_stockerPosition):
        AmbiDynamixelGoto.__init__(self, 
                                   [AmbiDynamixelCmd(p_side, "CannonFinger", p_fingerPosition),
                                    AmbiDynamixelCmd(p_side, "CannonStocker", p_stockerPosition)])