#!/usr/bin/env python


#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
import os       
     
class StateSelector(CyclicState):
    def __init__(self):
        CyclicState.__init__(self,outcomes=['1','2','3','4','9','10'])
        
    def executeWhile(self):
        self.button=Inputs.getManualButton()
        return
        
    def executeTransitions(self):
        if self.button == "1" or self.button == "2" or self.button == "3" or self.button == "4" or self.button == "9" or self.button == "10":
            return self.button
    
