#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs

from Inputs import Inputs
from Data import Data
from CyclicState import CyclicState

class WaiterState(CyclicState):
    def __init__(self,waitTime):
        CyclicState.__init__(self, outcomes=['done'])
        self.waitTime=waitTime
        
    def executeTransitions(self):
        if (rospy.get_rostime().secs-self.timeIn>self.waitTime):
            return 'done'

    def executeIn(self): 
        self.timeIn=rospy.get_rostime().secs  