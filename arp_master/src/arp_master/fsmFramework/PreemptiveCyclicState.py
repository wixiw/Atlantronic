#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs

from arp_master.util.Inputs import Inputs
from arp_master.util.Data import Data
from CyclicState import CyclicState

#you should derivate this class when you want to create a preemptive state
class PreemptiveCyclicState(CyclicState):
    #this function shall be overloaded, and return true when preemption is requested
    def preemptionCondition(self):
        return false
    
