#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs

from CyclicState import CyclicState
from Inputs import Inputs
from Data import Data
from Table2011 import *
from UtilARD import *

class Initialisation(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['endInitialisation'])
        with self:
            smach.StateMachine.add('Init', Init(),
                                   transitions={'initstateok':'WaitForStart'})
            smach.StateMachine.add('WaitForStart', WaitForStart(),
                                   transitions={'start':'endInitialisation'})
    
 
class Init(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['initstateok'])

    def executeTransitions(self):
       # the only condition verified to go on is that the start is not put
       if Inputs.getstart()==1:
           return 'initstateok'
        
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
        poseDepart=AmbiPoseRed(-0.9,0.84,-pi/2,Data.color)
        self.setPosition(poseDepart.x,poseDepart.y,poseDepart.theta)
        self.enableDrive()
    