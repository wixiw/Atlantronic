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
        self.PUMP_TEST_POWER = 20
        self.PUMP_TEST_DURATION = 2.0

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
                      transitions={'done':'WaitABit'})
            
            smach.StateMachine.add('WaitABit',
                      WaiterState(self.PUMP_TEST_DURATION),
                      transitions={'timeout':'StopLeftFingerPump'})
            
            smach.StateMachine.add('StopLeftFingerPump',
                      SendPumpCmd("LeftFingerPump", 0),
                      transitions={'done':'StopRightFingerPump'})
            
            smach.StateMachine.add('StopRightFingerPump',
                      SendPumpCmd("RightFingerPump", 0),
                      transitions={'done':'StopArmPump'})
            
            smach.StateMachine.add('StopArmPump',
                      SendPumpCmd("ArmPump", 0),
                      transitions={'done':'succeeded'})
            
            #