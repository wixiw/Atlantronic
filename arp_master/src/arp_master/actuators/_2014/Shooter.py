#!/usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master.strat_2014.common_2014.Robot2014 import *

from arp_master.fsmFramework import *
from arp_master.commonStates import *
from arp_master.strat_2014 import *
from arp_master.actuators import *

#TODO
##
##
## Remettre le preemptive

class ShooterMainStateMachine(PreemptiveStateMachine):
    def __init__(self, p_side):
        PreemptiveStateMachine.__init__(self,outcomes=['end','problem'])

        #Configuration of reference positions
        self.refPosStocker = { 'UP' : 0.0,
                              'DOWN' : 0.0}
        self.refPosShooter = { 'ARM' : 0.0,
                              'SHOOT' : 0.0,
                              'IDLE' : 0.0}
                
        with self:      

#Remplacer UserDebugTrigger par vraie etat une fois fini
                        
            PreemptiveStateMachine.add('Idle',
                      CannonBiCommand(p_side, self.refPosShooter['IDLE'], self.refPosStocker['UP']),
                      transitions={'succeeded':'WaitForStratCommand', 'problem':'problem'})
            
            PreemptiveStateMachine.add('WaitForStratCommand',
                      CannonWaitForStratRequest(p_side),
                      transitions={'load':'Load', 
                                   'shoot':'problem'})
            
            #Load
            PreemptiveStateMachine.add('Load',
                      CannonBiCommand(p_side, self.refPosShooter['ARM'], self.refPosStocker['DOWN']),
                      transitions={'succeeded':'WaitForBall', 'problem':'problem'})  
            
            PreemptiveStateMachine.add('WaitForBall',
                      WaiterState(1.0),
                      transitions={'timeout':'CloseStocker'})
            
            PreemptiveStateMachine.add('CloseStocker',
                      CannonBiCommand(p_side, self.refPosShooter['ARM'], self.refPosStocker['UP']),
                      transitions={'succeeded':'WaitForShoot', 'problem':'problem'})  
                        
            PreemptiveStateMachine.add('WaitForShoot',
                      CannonWaitForStratRequest(p_side),
                      transitions={'load':'WaitForShoot', 
                                   'shoot':'Shoot'})
                        
            #Shoot                        
            PreemptiveStateMachine.add('Shoot',
                      CannonBiCommand(p_side, self.refPosShooter['SHOOT'], self.refPosStocker['UP']),
                      transitions={'succeeded':'WaitABit', 'problem':'problem'})       
            
            PreemptiveStateMachine.add('WaitABit',
                      WaiterState(0.5),
                      transitions={'timeout':'Idle'})    
            

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
# Use this state to wait for a strat command
# Available commands are :
# _ load
# _ arm
# _ shoot
# @param String p_side : the side of the cannon on the robot
#
class CannonWaitForStratRequest(ReceiveFromTopic):
    def __init__(self, p_side):
        ReceiveFromTopic.__init__(self, "/Ubiquity/"+p_side+"Finger/strat_command", String,
                                  outcomes=['load','arm','shoot']) 
        
    def executeTransitions(self):
        #if no message has been received yet, just wait again
        if self.msg is None:
            return
        #end with success if last message is a known command
        if self.msg.data is 'load':
            return 'load'  
        if self.msg.data is 'arm':
            return 'arm' 
        if self.msg.data is 'shoot':
            return 'shoot' 
        
        