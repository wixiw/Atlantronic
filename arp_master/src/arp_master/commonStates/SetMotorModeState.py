#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *

class SetSteeringMotorModeState(CyclicState):
    def __init__(self, mode):
        CyclicState.__init__(self, outcomes=['succeeded','failed'])
        self.mode = mode
    
    def executeIn(self):
        self.result = self.setSteeringMotorMode(self.mode)
    
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
    
    def executeTransitions(self):
        if self.result == True:
            return 'succeeded'   
        else:
            return 'failed'