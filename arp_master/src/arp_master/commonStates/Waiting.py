#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
import os

#
# Those states allows the robot to wait for something.
# * WaiterState is waiting a timer to fired.
# * WaitForStart is waiting for a start to be plugged
# * WaitForMatch is waiting for a start unplug (will record start time)
############################################################

#the state is a waiting timer. Take care about waitTime=0 which is interpreted as a little timeout of 0.001.
#@param waitTime is in second
class WaiterState(CyclicState):
    def __init__(self,waitTime):
        CyclicState.__init__(self, outcomes=[])
        if waitTime == 0:
            self.timeout = 0.001
        else:
            self.timeout = waitTime
        
    def executeTransitions(self):
            return
        


#the state that will wait for the start to be pluged
class WaitForStart(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['start'])
    
    def executeIn(self):
        os.system("beep -f 300 -l300 -r2") 
    
    def executeTransitions(self):
       if Inputs.getstart()==0:
            return 'start'
        
#the state that will wait for the start to be unpluged (before the match or for tests)
class WaitForStartUnplug(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['startunplug'])
    
    def executeTransitions(self):
       if Inputs.getstart()==1:
            return 'startunplug'
        
#wait for start to be plugged out
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
        self.stopGyroCalibration(); 
