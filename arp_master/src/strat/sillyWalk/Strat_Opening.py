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

class Opening(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['endOpening','problem'])
        with self:
            smach.StateMachine.add('FinRush',
                      FinRush(),
                      transitions={'succeeded':'endOpening','aborted':'problem'})
 
class FinRush(CyclicActionState):
    def createAction(self):
        casedepose=AmbiCaseRed(-1,-5,Data.color)
        (xobj,yobj)=casedepose.coord_WhenPionMilieu(-pi/2)
        self.pointcap(xobj,yobj,-pi/2)