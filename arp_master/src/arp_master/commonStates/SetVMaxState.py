#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
import math

# 
# These states are usefull to set the pspeed of the robot
#
##################################################


class SetVMaxState(CyclicState):
    def __init__(self,vMax):
        CyclicState.__init__(self, outcomes=['succeeded'])
        self.v = vMax
            
    def executeIn(self):
        self.setVMax(self.v)

    def executeTransitions(self):
        return 'succeeded'
 