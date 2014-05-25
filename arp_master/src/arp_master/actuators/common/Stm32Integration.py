#!/usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
from arp_master.util import *
from arp_master.fsmFramework import *
from arp_master.commonStates import *
from std_msgs.msg import *

#
# You may drive a pump with this state. It send a single command, it is *not* periodic.
# @param pumpName : String - the name of the pump to drive
# @param suctionPower : Integer - drive the suction power in percent in the range [50;100]
#
class SendPumpCmd(SendOnTopic):
    def __init__(self, p_pumpName, p_suctionPower):
        SendOnTopic.__init__(self, "/Ubiquity/"+p_pumpName+"/suction_power", UInt8, UInt8(p_suctionPower))

#
# You may drive a dynamixel with this state. It send a single command, it is *not* periodic.
# Basically it means that you are *NOT* at the desired position when you exit this state
# @param dynamixelName : String - the name of the dynamixel to drive
# @param position : Double - position cmd in [-pi;pi] in radians
#
class DynamixelNonBlockingPositionCmd(SendOnTopic):
    def __init__(self, p_dynamixelName, p_position):
        SendOnTopic.__init__(self, "/Ubiquity/"+p_dynamixelName+"/position_cmd", Float32, Float32(p_position))

#
# You may configure a dynamixel torque with this state. It send a single command, it is *not* periodic.
# Basically it means that the torque cmd is just sent but not executed when you exit the state
# @param String - dynamixelName     : the name of the dynamixel to configure
# @param Percentage - torque        : the torque in % in [20;100]
#
class DynamixelTorqueConfig(SendOnTopic):
    def __init__(self, p_dynamixelName, p_torque):
        SendOnTopic.__init__(self, "/Ubiquity/"+p_dynamixelName+"/max_torque", UInt8, UInt8(p_torque))

#
# You may drive a dynamixel with this state. It send a single command, it is *not* periodic.
# Basically it means that you are *NOT* at the desired position when you exit this state
# The dynamixel has 2s to reach its position
# @param dynamixelName : String - the name of the dynamixel to drive
#
class WaitDynamixelReachedPosition(ReceiveFromTopic):
    def __init__(self, p_dynamixelName):
        ReceiveFromTopic.__init__(self, "/Ubiquity/"+p_dynamixelName+"/state", DynamixelState,
            p_outcomes=['pos_reached','stucked'], p_timeout=2.0)

    def executeTransitions(self):    
        #if no message has been received yet, just wait again
        if self.msg is None:
            return
        #end with success if the dynamixel is target_reached
        if self.msg.target_reached is True:
            return 'pos_reached' 
        #if the servo is stucked end with failure
        if self.msg.stucked is True:
            return 'stucked'
        #else : continue to wait
         
#    
# Use this state to send a blocking "goto" command to list of dynamixels
# The dynamixels has 2s (cumulative in the number of dynamixels) to reach their positions
# @param dynamixelName : List(String) - the name of the dynamixels to drive
# @param position : List(Double) - position cmd in [-pi;pi] in radians
#   
class DynamixelGoto(smach.StateMachine):
    def __init__(self,p_dynamixelNameList, p_positionList):
        smach.StateMachine.__init__(self,outcomes=['succeeded','problem'])
        with self: 
            for i,dynamixelName in enumerate(p_dynamixelNameList):
                stateName = 'SendCommandTo' + dynamixelName
                if i >= len(p_dynamixelNameList)-1:
                    transitionName = 'Wait'
                else:
                    transitionName = 'SendCommandTo' + p_dynamixelNameList[i+1]
                smach.StateMachine.add(stateName,
                    DynamixelNonBlockingPositionCmd(dynamixelName, p_positionList[i]),
                    transitions={ 'done' : transitionName })
        
            smach.StateMachine.add('Wait',
                    WaiterState(0.1),
                    transitions={'timeout':'Wait' + p_dynamixelNameList[0] + 'PositionReached'})
              
            for j,dynamixelName in enumerate(p_dynamixelNameList):   
                stateName = 'Wait' + dynamixelName + 'PositionReached'
                if i >= len(p_dynamixelNameList)-1:
                    transitionName = 'succeeded'
                else:
                    transitionName = 'Wait' + p_dynamixelNameList[j+1] + 'PositionReached'
                    
                smach.StateMachine.add(stateName,
                    WaitDynamixelReachedPosition(dynamixelName),
                    transitions={'pos_reached':transitionName, 'stucked':'problem', 'timeout':'problem'})
        
#
# Use this state to wait an Omron to be in an expected state
# @param p_omronName : String - the omron to listen
# @param p_awaitedValue : Boolean - 'True' or 'False' The value you are waiting until the timeout.
#
class WaitForOmronValue(ReceiveFromTopic):
    def __init__(self, p_omronName, p_awaitedValue, p_timeout):
        ReceiveFromTopic.__init__(self, "/Ubiquity/"+p_omronName+"/object_present", Bool,
                                  outcomes=['triggered'], timeout=p_timeout) 

        #remember reference value
        self.awaitedValue = p_awaitedValue
        
    def executeTransitions(self):
        #if no message has been received yet, just wait again
        if self.msg is None:
            return
        #end with success if last message is the awaited value, else continue to wait
        if self.msg.data == self.awaitedValue:
            return 'triggered'  