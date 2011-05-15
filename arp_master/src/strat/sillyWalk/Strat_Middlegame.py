#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs

from CyclicState import CyclicState
from CyclicActionState import CyclicActionState
from Inputs import Inputs
from Data import Data
from arp_ods.msg import OrderGoal
from arp_ods.msg import OrderAction

from math import pi

from Table2011 import *
from UtilARD import *

class Middlegame(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['endMiddlegame'])
        with self:
            smach.StateMachine.add('CreateWalk',
                      CreateWalk(),
                      transitions={'toStep1':'Step1','toStep2':'Step2','toStep3':'Step3'})
            smach.StateMachine.add('Step1',
                      Step1(),
                      transitions={'succeeded':'Step2','aborted':'CreateWalk'})
            smach.StateMachine.add('Step2',
                      Step2(),
                      transitions={'succeeded':'Step3','aborted':'CreateWalk'})
            smach.StateMachine.add('Step3',
                      Step3(),
                      transitions={'succeeded':'CreateWalk','aborted':'CreateWalk'})


class CreateWalk(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['toStep1','toStep2','toStep3'])
    
    def executeTransitions(self):
            return 'toStep1'
 
class Step1(CyclicActionState):
    def createAction(self):
       self.pointcap(0.5,0.5,0)

class Step2(CyclicActionState):
    def createAction(self):
       self.pointcap(0.5,-0.5,-pi/2)

class Step3(CyclicActionState):
    def createAction(self):
       self.pointcap(-0.5,-0.5,pi)      
       