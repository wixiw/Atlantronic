#!/usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
from arp_master.fsmFramework import *


#Log a text
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
            
class PrintPosition(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['continue'])
    
    def executeIn(self):
        rospy.loginfo("(DEBUG] *** ---- Current Position : (" + str(Inputs.getx()) + "," + str(Inputs.gety()) + ","+ str(Inputs.gettheta()) + ")")
    
    def executeTransitions(self):
        return 'continue'  