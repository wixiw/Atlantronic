#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs

from Inputs import Inputs
from Data import Data
from PreemptiveCyclicState import PreemptiveCyclicState

class EndMatchPreempter(PreemptiveCyclicState):
    def __init__(self,offset):
        PreemptiveCyclicState.__init__(self, outcomes=['endMatch'])
        self.match_duration=rospy.get_param("/match_duration")+offset

    def preemptionCondition(self):
        if (rospy.get_rostime()-Data.start_time).to_sec()>self.match_duration:
            return True
        else:
            return False
       
    def executeTransitions(self):
        return 'endMatch'