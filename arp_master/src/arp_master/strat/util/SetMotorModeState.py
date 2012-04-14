#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs

from Inputs import Inputs
from Data import Data
from CyclicState import CyclicState

class SetSteeringMotorModeState(CyclicState):
    def __init__(self, mode):
        CyclicState.__init__(self, outcomes=['succeeded','failed'])
        self.mode = mode
    
    def executeIn(self):
        self.result = self.setSteeringMotorMode(self.mode)
        rospy.loginfo("A CODER !!")
    
    def executeTransitions(self):
        if self.result == True:
            return 'succeeded'   
        else:
            return 'failed'
        
        
class SetDrivingMotorModeState(CyclicState):
    def __init__(self, mode):
        CyclicState.__init__(self, outcomes=['succeeded','failed'])
        self.mode = mode
    
    def executeIn(self):
        self.result = self.setDrivingMotorMode(self.mode)
        rospy.loginfo("A CODER !!")
    
    def executeTransitions(self):
        if self.result == True:
            return 'succeeded'   
        else:
            return 'failed'