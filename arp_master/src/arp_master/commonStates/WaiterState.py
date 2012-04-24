#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *

class WaiterState(CyclicState):
    def __init__(self,waitTime):
        CyclicState.__init__(self, outcomes=['done'])
        self.waitTime=waitTime
        
    def executeTransitions(self):
        if (rospy.get_rostime().secs-self.timeIn>self.waitTime):
            return 'done'

    def executeIn(self): 
        self.timeIn=rospy.get_rostime().secs  