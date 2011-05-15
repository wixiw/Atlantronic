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

#createWalk est le dispatcher
#step1 est le recul sur la case adverse
#step2 est l'avancee en diagonale sur les cases adverses
#step3 est la poussee du palet sur sa propre case

class CreateWalk(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['toStep1','toStep2','toStep3'])
    
    def executeTransitions(self):
        #mon role est de creer les ordres de mouvements pour un cycle de silly walk
        #je vais creer 3 cases de silly walk et envoyer dessus le robot
        Data.case1=Case(-1,-3)
        Data.case2=Case(5,3)
        Data.case3=Case(5,5)
        Data.angleParcours=pi/4
        return 'toStep1'
 
class Step1(CyclicActionState):
    def createAction(self):
        self.pointcap_reverse(Data.case1.xCenter,Data.case1.yCenter,Data.angleParcours)

class Step2(CyclicActionState):
    def createAction(self):
        self.pointcap(Data.case2.xCenter,Data.case2.yCenter,Data.angleParcours)

class Step3(CyclicActionState):
    def createAction(self):
        angleobj=Data.angleParcours+pi/4
        (xobj,yobj)=Data.case3.coord_WhenPionMilieu(angleobj)
        self.pointcap(xobj,yobj,angleobj)

       