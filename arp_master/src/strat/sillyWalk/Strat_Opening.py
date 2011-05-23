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
        smach.StateMachine.__init__(self, outcomes=['endOpening', 'problem'])
        with self:
            smach.StateMachine.add('EscapeStartpoint',
                      EscapeStartpoint(),
                      transitions={'succeeded':'ChopePaletMilieu', 'aborted':'problem'})
            smach.StateMachine.add('ChopePaletMilieu',
                      ChopePaletMilieu(),
                      transitions={'succeeded':'Depose', 'aborted':'problem'})
            smach.StateMachine.add('Depose',
                      Depose(),
                      transitions={'succeeded':'PrepareSillyWalk', 'aborted':'problem'})
            smach.StateMachine.add('PrepareSillyWalk',
                      PrepareSillyWalk(),
                      transitions={'succeeded':'endOpening', 'aborted':'problem'})
 
class EscapeStartpoint(CyclicActionState):
    def createAction(self):
        pose = AmbiPoseRed(-0.5, Table.HWALL_Y-0.200,0, Data.color)
        self.pointcap_pass(pose.x, pose.y, pose.theta)

class ChopePaletMilieu(CyclicActionState):
    def createAction(self):
        casedepose = AmbiCaseRed(0, 0, Data.color)
        (xobj, yobj) = casedepose.coord_WhenPionMilieu(-pi / 2)
        self.pointcap_pass(xobj, yobj, -pi / 2)           
        
class Depose(CyclicActionState):
    def createAction(self):
        casedepose = AmbiCaseRed(-1, -5, Data.color)
        
        if Data.color=='red':
            (xobj, yobj) = casedepose.coord_WhenPionMilieu(-pi / 2)
            yobj+=0.045
            xobj-=0.080
            capobj=-pi / 2
        else:
            (xobj, yobj) = casedepose.coord_WhenPionMilieu(-pi / 2+pi/8)
            capobj=-pi / 2+pi/8
            yobj+=0.045
            xobj-=0.080

        self.pointcap(xobj, yobj, capobj)
      

        
class PrepareSillyWalk(CyclicActionState):
    def createAction(self):
        case = AmbiCaseRed(-1, -3, Data.color)
        self.pointcap_reverse(case.xCenter-0.100, case.yCenter, pi/8)
