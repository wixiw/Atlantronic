#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs

from arp_master.strat.util.CyclicState import CyclicState
from arp_master.strat.util.Inputs import Inputs
from arp_master.strat.util.Data import Data
from arp_master.strat.util.TableVierge import *


# the main state machine
# it is composed of 2 states: init and wait for start
class Initialisation(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['endInitialisation'])
        with self:
            smach.StateMachine.add('Init', Init(),
                                   transitions={'initstateok':'WaitForStart'})
            smach.StateMachine.add('WaitForStart', WaitForStart(),
                                   transitions={'start':'endInitialisation'})
    
#the first state: only to wait for the start to be unpluged 
class Init(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['initstateok'])

    def executeTransitions(self):
       # the only condition verified to go on is that the start is not put
       if Inputs.getstart()==1:
           return 'initstateok'

#the state that will wait for the start to be pluged
class WaitForStart(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['start'])
    
    def executeTransitions(self):
       if Inputs.getstart()==0:
            return 'start'
        
    def executeOut(self):
        Data.color=Inputs.getcolor()
        if Data.color=='red':
            Data.adv_color='blue'
        else:
            Data.adv_color='red'
        poseDepart=AmbiPoseRed(1.250,0.750,-pi,Data.color)
        self.setPosition(poseDepart.x,poseDepart.y,poseDepart.theta)
            
    