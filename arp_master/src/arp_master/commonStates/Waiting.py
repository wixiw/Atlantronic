#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
import os
from arp_master.fsmFramework import *

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
        os.system("beep -f 200 -l200 -r2") 
    
    def executeTransitions(self):
       if Inputs.getstart()==1:
            return 'start'
        
#the state that will wait for the start to be unpluged (before the match or for tests)
class WaitForStartUnplug(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['startunplug'])
    
    def executeIn(self):
        os.system("beep -f 200 -l100 -r1") 
        
    def executeTransitions(self):
       if Inputs.getstart()==0:
            return 'startunplug'
        
#wait for start to be plugged out
class WaitForMatch(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['start'])
    
    def executeIn(self):
        os.system("beep -f 300 -l300 -r3")
    
    def executeTransitions(self):
       if Inputs.getstart()==0:
            return 'start'
        
    def executeOut(self):
        #je note le temps de debut de match
        Data.start_time=rospy.get_rostime()
        self.stopGyroCalibration(); 

#wait for start to be plugged out
class WaitForMatch(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['start'])
    
    def executeIn(self):
        os.system("beep -f 300 -l300 -r3")
    
    def executeTransitions(self):
       if Inputs.getstart()==0:
            return 'start'
        
    def executeOut(self):
        #je note le temps de debut de match
        Data.start_time=rospy.get_rostime()
        self.stopGyroCalibration(); 

#wait for start to be plugged out
class LoggerState(CyclicState):
    def __init__(self,text):
        self.text = text
        CyclicState.__init__(self, outcomes=['continue'])
    
    def executeIn(self):
        rospy.loginfo(self.text)
    
    def executeTransitions(self):
        return 'continue'

# Debug state : print a log and ask a strat plug in/plug out
class UserDebugTrigger(smach.StateMachine):
    def __init__(self,text):
        smach.StateMachine.__init__(self, outcomes=['continue'])
        with self:
            smach.StateMachine.add('Log', 
                                   LoggerState(text),
                                   transitions={'continue':'WaitForStart', 'timeout':'Log'})
            smach.StateMachine.add('WaitForStart', 
                                   WaitForStart(),
                                   transitions={'start':'WaitForStartUnplug', 'timeout':'Log'})  
            smach.StateMachine.add('WaitForStartUnplug', 
                                   WaitForStartUnplug(),
                                   transitions={'startunplug':'continue', 'timeout':'Log'})  
