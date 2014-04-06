#!/usr/bin/env python


#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
import os
from SetPosition import *
from Waiting import *
from arp_master.fsmFramework import *

#
# This is the default a1 level state for any strategy. Except for testing purpose it should be overided in any strategy
#
##################################################


class StartSequence(smach.StateMachine):
    def __init__(self,x,y,theta):
        smach.StateMachine.__init__(self,outcomes=['gogogo','problem'])
        with self:
            smach.StateMachine.add('SetInitialPosition',
                      SetInitialPosition(x,y,theta),
                      transitions={'succeeded':'StartGyroCalibration', 'timeout':'problem'})
            #SetInitial position se charge aussi du gyro.

            smach.StateMachine.add('StartGyroCalibration',
                      StartGyroCalibrationState(),
                      transitions={'succeeded':'WaitForMatch', 'timeout':'WaitForMatch'})
                        
            smach.StateMachine.add('WaitForMatch', 
                      WaitForMatch(),
                      transitions={'start':'gogogo', 'timeout':'problem'})
            #calibration is stopped by WaitForMatch
