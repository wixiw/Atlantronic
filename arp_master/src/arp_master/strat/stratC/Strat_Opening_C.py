#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs

from arp_master.strat.util.CyclicState import CyclicState
from arp_master.strat.util.CyclicActionState import CyclicActionState
from arp_master.strat.util.PreemptiveStateMachine import PreemptiveStateMachine
from arp_master.strat.util.PreemptiveCyclicState import PreemptiveCyclicState
from arp_master.strat.util.Inputs import Inputs
from arp_master.strat.util.Data import Data
from arp_ods.msg import OrderGoal
from arp_ods.msg import OrderAction

from math import pi

from arp_master.strat.util.Table2011 import *
from arp_master.strat.util.UtilARD import *

class Opening_C(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self, outcomes=['endOpening', 'problem'])
        with self:
            PreemptiveStateMachine.addPreemptive('ObstaclePreemption',
                      ObstaclePreemption(),
                      transitions={'avoid':'EscapeObstacle'})
            
            PreemptiveStateMachine.add('EscapeStartpoint',
                      EscapeStartpoint(),
                      transitions={'succeeded':'ChopePaletMilieu', 'aborted':'problem'})
            
            self.setInitialState('EscapeStartpoint')
            
            PreemptiveStateMachine.add('ChopePaletMilieu',
                      ChopePaletMilieu(),
                      transitions={'succeeded':'Depose', 'aborted':'problem'})
            
            PreemptiveStateMachine.add('Depose',
                      Depose(),
                      transitions={'succeeded':'PrepareSillyWalk', 'aborted':'problem'})
            PreemptiveStateMachine.add('PrepareSillyWalk',
                      PrepareSillyWalk(),
                      transitions={'succeeded':'endOpening', 'aborted':'problem'})
            
            PreemptiveStateMachine.add('EscapeObstacle',
                      EscapeObstacle(),
                      transitions={'succeeded':'problem', 'aborted':'problem'})
 
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
        self.pointcap_reverse(case.xCenter, case.yCenter, pi/4)
        

class ObstaclePreemption(PreemptiveCyclicState):
    def __init__(self):
        PreemptiveCyclicState.__init__(self, outcomes=['avoid'])
        self.blinding_period=rospy.get_param("/blinding_period")

    def preemptionCondition(self):
        if Inputs.getobstacle()==1 and rospy.get_rostime().secs-Data.time_obstacle>self.blinding_period:
            Data.time_obstacle=rospy.get_rostime().secs
            return True
        else:
            return False
       
    def executeTransitions(self):
        return 'avoid'

        
class EscapeObstacle(CyclicActionState):
    def createAction(self):
        case = AmbiCaseRed(-3, 3, Data.color)
        cap = AmbiCapRed(0,Data.color)
        self.pointcap_reverse(case.xCenter, case.yCenter, cap.angle)