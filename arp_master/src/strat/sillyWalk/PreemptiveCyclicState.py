#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs

from Inputs import Inputs
from Data import Data

#you should derivate this class when you want to creat a preemptive state
class PreemptiveCycliState(CyclicState):
    #this function shall be overloaded, and return true when preemption is requested
    def preemptionCondition(self):
        return false
    
