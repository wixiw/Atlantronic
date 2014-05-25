#!/usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master.fsmFramework import *
from arp_master.commonStates import *
from arp_master.actuators.common.Stm32Integration import *

class SelfTest(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['succeeded','problem'])  
        
        # Definition of Pump test parameters
        self.PUMP_TEST_POWER = 50
        self.STRESS_TEST_DURATION = 1
        
        #Definition of servo test parameters
        self.DYNAMIXEL_DELTA = 0.7
        self.dynamixelList = [                       
                             'LeftCannonFinger',
                             'LeftCannonStocker',
                             'RightCannonFinger',
                             'RightCannonStocker',
                             'LeftFinger',
                             'RightFinger',
                             'ArmSlider',
                             'ArmShoulder',
                             'ArmShoulderElbow',
                             'ArmWristElbow',
                             'ArmWrist'
                             ]

        with self:
            #Launch the 3 pumps and wait 2 seconds
            smach.StateMachine.add('TryLeftFingerPump',
                      SendPumpCmd("LeftFingerPump", self.PUMP_TEST_POWER),
                      transitions={'done':'TryRightFingerPump'})
            
            smach.StateMachine.add('TryRightFingerPump',
                      SendPumpCmd("RightFingerPump", self.PUMP_TEST_POWER),
                      transitions={'done':'TryArmPump'})
            
            smach.StateMachine.add('TryArmPump',
                      SendPumpCmd("ArmPump", self.PUMP_TEST_POWER),
                      transitions={'done':'Move'+self.dynamixelList[0]})
            
            #Move all dynamixels
            for index, dynamixelName in enumerate(self.dynamixelList):
                stateName = 'Move' + dynamixelName
                if index >= len(self.dynamixelList)-1:
                    transitionName = 'WaitABit'
                else:
                    transitionName = 'Move' + self.dynamixelList[index+1]
                    
                smach.StateMachine.add(stateName,
                          DynamixelNonBlockingPositionCmd(dynamixelName, self.DYNAMIXEL_DELTA),
                          transitions={'done': transitionName})
                
            smach.StateMachine.add('WaitABit',
                      WaiterState(self.STRESS_TEST_DURATION),
                      transitions={'timeout':'StopLeftFingerPump'})
            
            smach.StateMachine.add('StopLeftFingerPump',
                      SendPumpCmd("LeftFingerPump", 0),
                      transitions={'done':'StopRightFingerPump'})
            
            smach.StateMachine.add('StopRightFingerPump',
                      SendPumpCmd("RightFingerPump", 0),
                      transitions={'done':'StopArmPump'})
            
            smach.StateMachine.add('StopArmPump',
                      SendPumpCmd("ArmPump", 0),
                      transitions={'done':'Move'+self.dynamixelList[0] +'2'})
            
            for index, dynamixelName in enumerate(self.dynamixelList):
                stateName = 'Move' + dynamixelName +'2'
                if index >= len(self.dynamixelList)-1:
                    transitionName = 'succeeded'
                else:
                    transitionName = 'Move' + self.dynamixelList[index+1] +'2'
                    
                smach.StateMachine.add(stateName,
                          DynamixelNonBlockingPositionCmd(dynamixelName, 0.0),
                          transitions={'done': transitionName})
