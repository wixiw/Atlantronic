#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *

# This state allows you to check if the end match timer is fired. It should be present in *every* operationnal state machine
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