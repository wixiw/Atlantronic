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
        try:
            self.setVMax(self.v)
        except Exception:
            rospy.logerr("SETVMAXSTATE: j'ai pas reussi a faire passer le SetVMax mais bon... continuons !") 
            

    def executeTransitions(self):
        return 'succeeded'
 