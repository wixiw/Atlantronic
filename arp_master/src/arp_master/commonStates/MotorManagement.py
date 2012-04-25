#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *

# 
# These states are usefull to set or disable the power on motors
#
##################################################

# Steering motion mode selector : mode = "speed" or "position" or "homing" or "torque" or "other"
# normalement on utilise que "position"
# pour realiser un homing se referer a la fonction FindSteeringZero
class SetSteeringMotorModeState(CyclicState):
    def __init__(self, mode):
        CyclicState.__init__(self, outcomes=['succeeded','timeout'])
        self.mode = mode
    
    def executeIn(self):
        self.result = self.setSteeringMotorMode(self.mode)
    
    def executeTransitions(self):
        if self.result == True:
            return 'succeeded'   
        else:
            return 'timeout'
        
# Permet de faire un homing sur les moteurs de directions pour definir les 0
# Pensez a mettre la puissance steering avant, et a repasser en mode position apres :D
class FindSteeringZero(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['succeeded','timeout'])
        self.timeout = 10
        self.srvResult = False

    
    def executeIn(self):
        self.srvResult = self.setSteeringMotorMode("homing")
        
    def executeTransitions(self):
        if self.srvResult == False:
            return 'timeout'
        if self.srvResult == True and Inputs.gethomingdone() == True:
            return 'succeeded'   
        
# Driving motion mode selector : mode = "speed" or "position" or "homing" or "torque" or "other"
# normalement on utilise que "speed" et "torque"
class SetDrivingMotorModeState(CyclicState):
    def __init__(self, mode):
        CyclicState.__init__(self, outcomes=['succeeded','timeout'])
        self.mode = mode
    
    def executeIn(self):
        self.result = self.setDrivingMotorMode(self.mode)
    
    def executeTransitions(self):
        if self.result == True:
            return 'succeeded'   
        else:
            return 'timeout'
        
        
#Steering power management            
class SetSteeringPower(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['succeeded','timeout'])
    
    def executeIn(self):
        self.result = self.enableSteeringPower()
    
    def executeTransitions(self):
        if self.result == True:
            return 'succeeded'   
        else:
            return 'timeout'   

#Driving power management       
class SetDrivingPower(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['succeeded','timeout'])
    
    def executeIn(self):
        self.result = self.enableDrivingPower()
    
    def executeTransitions(self):
        if self.result == True:
            return 'succeeded'   
        else:
            return 'timeout' 
        
        
        
               
  
