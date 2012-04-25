#!/usr/bin/env python


#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
import os
from Waiting import *

#
# This is the default a0 level state in any strategy. 
# _ It checks if the initial state is as expected
# _ It waits for Orocos to be up
# _ it waits for the start to be plugged in
#
##################################################

class Initialisation(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['endInitialisation','failed'])
        with self:
            smach.StateMachine.add('Init', Init(),
                                   transitions={'initstateok':'WaitForOrocos','timeout':'failed'})
            smach.StateMachine.add('WaitForOrocos', 
                                   WaitForOrocos(),
                                   transitions={'deployed':'WaitForStart','timeout':'failed'})
            smach.StateMachine.add('WaitForStart', 
                                   WaitForStart(),
                                   transitions={'start':'endInitialisation','timeout':'failed'})
    
    
#the first state: only to wait for the start to be unpluged 
class Init(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['initstateok'])

    def executeTransitions(self):
       # the only condition verified to go on is that the start is not put
       if Inputs.getstart()==1:
           return 'initstateok'

#the state that will wait orocos software to deploy
class WaitForOrocos(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['deployed'])
    
    def executeTransitions(self):
       if Inputs.getdeployed() is True:
            return 'deployed'
    
    def executeOut(self):
        rospy.loginfo("Orocos deployed.")

            
    