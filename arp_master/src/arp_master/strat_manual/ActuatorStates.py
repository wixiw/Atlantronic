#!/usr/bin/env python


#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
import os       
       
     
class State1(CyclicState):
    def __init__(self):
        CyclicState.__init__(self,outcomes=['2','3','4','9','10'])
        
    def executeWhile(self):
        self.button=Inputs.getManualButton()
        return
        
    def executeTransitions(self):
        if self.button == "2" or self.button == "3" or self.button == "4" or self.button == "9" or self.button == "10":
            return self.button
    
    
class State2(CyclicState):
    def __init__(self):
        CyclicState.__init__(self,outcomes=['1','3','4','9','10'])
        
    def executeWhile(self):
        self.button=Inputs.getManualButton()
        return
        
    def executeTransitions(self):
        if self.button == "1" or self.button == "3" or self.button == "4" or self.button == "9" or self.button == "10":
            return self.button
    
    
class State3(CyclicState):
    def __init__(self):
        CyclicState.__init__(self,outcomes=['1','2','4','9','10'])
        
    def executeWhile(self):
        self.button=Inputs.getManualButton()
        return
        
    def executeTransitions(self):
        if self.button == "1" or self.button == "2" or self.button == "4" or self.button == "9" or self.button == "10":
            return self.button
    
class State4(CyclicState):
    def __init__(self):
        CyclicState.__init__(self,outcomes=['1','2','3','9','10'])
        
    def executeWhile(self):
        self.button=Inputs.getManualButton()
        return
        
    def executeTransitions(self):
        if self.button == "1" or self.button == "2" or self.button == "3" or self.button == "9" or self.button == "10":
            return self.button
    
class State9(CyclicState):
    def __init__(self):
        CyclicState.__init__(self,outcomes=['1','2','3','4','10'])
        
    def executeWhile(self):
        self.button=Inputs.getManualButton()
        return
        
    def executeTransitions(self):
        if self.button == "1" or self.button == "2" or self.button == "3" or self.button == "4" or self.button == "10":
            return self.button
    
class State10(CyclicState):
    def __init__(self):
        CyclicState.__init__(self,outcomes=['1','2','3','4','9'])
        
    def executeWhile(self):
        self.button=Inputs.getManualButton()
        return
        
    def executeTransitions(self):
        if self.button == "1" or self.button == "2" or self.button == "3" or self.button == "4" or self.button == "9":
            return self.button