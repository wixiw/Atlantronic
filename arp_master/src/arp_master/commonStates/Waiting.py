#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
import os

#
# Those states allows the robot to wait for something.
# * WaiterState is waiting a timer to fired.
# * WaitForMatch is waiting for a start unplug (will record start time)
############################################################

class WaiterState(CyclicState):
    def __init__(self,waitTime):
        CyclicState.__init__(self, outcomes=[])
        self.timeout = waitTime
        
        
class WaitForMatch(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['start'])
    
    def executeIn(self):
        os.system("beep -f 300 -l300 -r3") 
    
    def executeTransitions(self):
       if Inputs.getstart()==1:
            return 'start'
        
    def executeOut(self):
        #je note le temps de debut de match
        Data.start_time=rospy.get_rostime()

