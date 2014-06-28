#!/usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master.fsmFramework import *
from arp_master.commonStates import *
from arp_master.actuators.common.Stm32Integration import *
from arp_master.actuators._2014.Shooter import *
from arp_master.strat_2014.common_2014.Robot2014 import *


class SelfTest(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['done'])  
        
        # Definition of Pump test parameters
        self.PUMP_TEST_POWER = 100
        self.STRESS_TEST_DURATION = 1
        
        with self:
            smach.StateMachine.add('TryLeftFingerPump',
                      SendPumpCmd("LeftFingerPump", self.PUMP_TEST_POWER),
                      transitions={'done':'TryRightFingerPump'})
            
            smach.StateMachine.add('TryRightFingerPump',
                      SendPumpCmd("RightFingerPump", self.PUMP_TEST_POWER),
                      transitions={'done':'TryArmPump'})
            
            smach.StateMachine.add('TryArmPump',
                      SendPumpCmd("ArmPump", self.PUMP_TEST_POWER),
                      transitions={'done':'TestDynamixel'})
            
            smach.StateMachine.add('TestDynamixel',
                      SelfTestDynamixelShowReady(),
                      transitions={'succeeded':'SelfTestSpeedDynamixel','problem':'SelfTestSpeedDynamixel'})
            
            smach.StateMachine.add('SelfTestSpeedDynamixel',
                      SelfTestSpeedDynamixel(),
                      transitions={'done':'WaitABit'})
            
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
                      transitions={'done':'DefaultDynamixelState'})
            
            smach.StateMachine.add('DefaultDynamixelState',
                       DefaultDynamixelState(),
                       transitions={'succeeded':'DefaultSpeedDynamixel', 'problem':'DefaultSpeedDynamixel'}) 
            
            smach.StateMachine.add('DefaultSpeedDynamixel',
                      DefaultSpeedDynamixel(),
                      transitions={'done':'done'})
    
    
class SelfTestSpeedDynamixel(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['done'])  
                
        with self:    
            smach.StateMachine.add('StartLeftShooter',
                   FingerSpeedCmd("Left", Robot2014.cannonFingerSpeed['SHOOT']),
                   transitions={'done':'StartRightShooter'})
            
            smach.StateMachine.add('StartRightShooter',
                   FingerSpeedCmd("Right", -Robot2014.cannonFingerSpeed['SHOOT']),
                   transitions={'done':'done'})

class DefaultSpeedDynamixel(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['done'])  
                
        with self:    
            smach.StateMachine.add('StopLeftShooter',
                   FingerSpeedCmd("Left", Robot2014.cannonFingerSpeed['STOPPED']),
                   transitions={'done':'StopRightShooter'})
            
            smach.StateMachine.add('StopRightShooter',
                   FingerSpeedCmd("Right", Robot2014.cannonFingerSpeed['STOPPED']),
                   transitions={'done':'done'})
        
#
# This state moves all dynamixels to show that they are alive
#
class SelfTestDynamixelShowReady(DynamixelGoto):
        def __init__(self):
            DynamixelGoto.__init__(self, Robot2014.dynamixelList,
                                   [ Robot2014.fingerLeftYellowPos['TORCH'],
                                    -Robot2014.fingerLeftYellowPos['TORCH'],
                                     Robot2014.cannonStockerLeftYellowPos['SHOWREADY'],
                                    -Robot2014.cannonStockerLeftYellowPos['SHOWREADY']
                                    ])


#
# This state machine allows to put any dynamixel in its initial state.
#
class DefaultDynamixelState(DynamixelGoto):
    def __init__(self):
        DynamixelGoto.__init__(self, Robot2014.dynamixelList, 
                                   [Robot2014.fingerLeftYellowPos['UP'],
                                    -Robot2014.fingerLeftYellowPos['UP'],
                                    Robot2014.cannonStockerLeftYellowPos['LOADING'],
                                    -Robot2014.cannonStockerLeftYellowPos['LOADING']
                                    ])
