#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs

from Inputs import Inputs
from Data import Data

#use this state machine if you want to have preemptive states within
# a preemptive state should only implement "preemptionCondition" function
# this PreemptiveStateMachine will register the preemptive state within the normal states, so that preemptive states call their preemption function
class PreemptiveStateMachine(smach.StateMachine):
    def __init__(self,outcomes):
        preemptiveStates=[]
        StateMachine.add(self,outcomes)
    
    #use this function to add a preemptive state
    def addPreemptive(name,state,transitions):
        self.preemptiveStates.append([name,state])
        StateMachine.add(name,state,transitions)
        
    #use this function to add a normal state
    def add(name,state,transitions):
        #register preemptive states
        state.preemptiveStates=self.preemptiveStates
        #extend the transitions ("auto transitions")
        for p in self.preemptiveStates:
            # I add a transition "preemptiveStatename" going into "preemtivestatename"
            transitions.add({p[0]:p[0]})
        #now I call the usual add function
        StateMachine.add(name,state,transitions)
    

    
    

    