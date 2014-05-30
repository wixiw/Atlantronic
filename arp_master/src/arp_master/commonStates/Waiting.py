#!/usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
from arp_master.fsmFramework import *
from arp_core.msg import Beep

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
        self.pubBeep = rospy.Publisher('/Master/beep', Beep)
    
    def executeIn(self):
        self.pubBeep.publish(Beep(200,0.2,2))
    
    def executeTransitions(self):
       if Inputs.getstart()==1:
            return 'start'
        
#the state that will wait for the start to be unpluged (before the match or for tests)
class WaitForStartUnplug(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['startunplug'])
        self.pubBeep = rospy.Publisher('/Master/beep', Beep)
    
    def executeIn(self):
        self.pubBeep.publish(Beep(200,0.1,1))
        
    def executeTransitions(self):
       if Inputs.getstart()==0:
            return 'startunplug'
        
#wait for start to be plugged out
class WaitForMatch(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['start'])
        self.pubBeep = rospy.Publisher('/Master/beep', Beep)
        self.pubReady = rospy.Publisher('/Master/ready_for_match', Bool)
    
    def executeIn(self):
        self.pubBeep.publish(Beep(300,0.3,3))
        self.pubReady.publish(Bool(True))
    
    def executeTransitions(self):
       if Inputs.getstart()==0:
            return 'start'
       
    def executeOut(self):
        #je note le temps de debut de match
        Data.start_time=rospy.get_rostime()
        self.stopGyroCalibration(); 

