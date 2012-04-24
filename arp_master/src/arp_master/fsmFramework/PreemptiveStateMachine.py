#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs

from arp_master.util.Inputs import Inputs
from arp_master.util.Data import Data

#use this state machine if you want to have preemptive states within
# a preemptive state should only implement "preemptionCondition" function
# this PreemptiveStateMachine will register the preemptive state within the normal states, so that preemptive states call their preemption function
class PreemptiveStateMachine(smach.StateMachine):
    def __init__(self,outcomes):
        self.preemptiveStates=[]
        smach.StateMachine.__init__(self,outcomes=outcomes)
    
    #use this function to add a preemptive state
    @staticmethod
    def addPreemptive(label,state,transitions=None,remapping=None):
        #je trouve cette implementation ridicule (cf ligne suivante). faut croire que c'est a cause du "with"
        #que add est une fonction static
        self=PreemptiveStateMachine._currently_opened_container()
        self.preemptiveStates.append([label,state])
        smach.StateMachine.add(label,state,transitions=transitions,remapping=remapping)
        
    #use this function to add a normal state
    @staticmethod
    def add(label,state,transitions=None,remapping=None):
        self=PreemptiveStateMachine._currently_opened_container()
        #register preemptive states
        state.preemptiveStates=self.preemptiveStates
        #extend the transitions ("auto transitions")
        for p in self.preemptiveStates:
            # I add a transition "preemptiveStatename" going into "preemtivestatename"
            transitions[p[0]]=p[0]
            # I must add this outcome to the state. initial outcomes were created by at initialisation of state
            state._outcomes.add(p[0])
        #now I call the usual add function
        smach.StateMachine.add(label,state,transitions=transitions,remapping=remapping)
    
    def setInitialState(self,label):
        self._initial_state_label=label
    
    

    