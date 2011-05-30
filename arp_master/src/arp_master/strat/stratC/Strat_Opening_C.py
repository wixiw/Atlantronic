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
                      transitions={'avoid':'problem'})
            
            PreemptiveStateMachine.add('EscapeStartpoint',
                      EscapeStartpoint(),
                      transitions={'succeeded':'Prempion1', 'aborted':'problem'})
            
            self.setInitialState('EscapeStartpoint')
            
            PreemptiveStateMachine.add('Prempion1',
                      Prempion1(),
                      transitions={'succeeded':'Prempion2', 'aborted':'problem'})
            
            PreemptiveStateMachine.add('Prempion2',
                      Prempion2(),
                      transitions={'succeeded':'Prempion3', 'aborted':'problem'})
            
            PreemptiveStateMachine.add('Prempion3',
                      Prempion3(),
                      transitions={'succeeded':'Prempion4', 'aborted':'problem'})
            
            PreemptiveStateMachine.add('Prempion4',
                      Prempion4(),
                      transitions={'succeeded':'RetourPrempion1', 'aborted':'problem'})
            
            PreemptiveStateMachine.add('RetourPrempion1',
                      RetourPrempion1(),
                      transitions={'succeeded':'RetourPrempion2', 'aborted':'problem'})
            
            PreemptiveStateMachine.add('RetourPrempion2',
                      RetourPrempion2(),
                      transitions={'succeeded':'PremRecal', 'aborted':'problem'})
            
            PreemptiveStateMachine.add('PremRecal',
                      PremRecal(),
                      transitions={'done':'PourWilly'})
            
            PreemptiveStateMachine.add('PourWilly',
                      PourWilly(),
                      transitions={'succeeded':'endOpening', 'aborted':'problem'})
 
 
class EscapeStartpoint(CyclicActionState):
    def createAction(self):
        pose = AmbiPoseRed(-0.8, Table.HWALL_Y-0.180,0, Data.color)
        self.pointcap(pose.x, pose.y, pose.theta)

class Prempion1(CyclicActionState):
    def createAction(self):
        pose = AmbiPoseRed(-0.8, 0.5, -pi/2, Data.color)
        if Data.color=='red':
            self.pointcap(pose.x, pose.y, pose.theta)
        else:
            self.pointcap(pose.x, pose.y-0.030, pose.theta)

class Prempion2(CyclicActionState):
    def createAction(self):
        if Data.color=='red':
            prempion=PionBord(1,'red')
            cap=-5*pi/6
            (xrobot,yrobot)=prempion.coord_WhenPionMilieu(cap)
            # j'ajoute -20 mm pour que ca soit un peu plus bas ca marche mieux..
            self.pointcap(xrobot-0.020,yrobot-0.020, cap )
        else:
            prempion=PionBord(1,'blue')
            cap=0
            (xrobot,yrobot)=prempion.coord_WhenPionMilieu(cap)
            self.pointcap(xrobot,yrobot, cap )
        
class Prempion3(CyclicActionState):
    def createAction(self):
        self.cap(-pi/8)            

class Prempion4(CyclicActionState):
    def createAction(self):
        case=AmbiCaseRed(-3,1,Data.color)
        #cap=AmbiCapRed(-pi/8,Data.color)
        #(xrobot,yrobot)=case.coord_WhenPionMilieu(cap.angle)
        #self.pointcap(xrobot, yrobot, cap.angle)   
        
        #>>>>>>>>>> retour devrait etre teste a true ou false !
        self.dropOnCase(case)  

class RetourPrempion1(CyclicActionState):
    def createAction(self):
        self.backward(0.2) 

class RetourPrempion2(CyclicActionState):
    def createAction(self):
        cap=AmbiCapRed(-pi,Data.color)
        self.cap(cap.angle)
        
class PremRecal(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['done'])
    
    def executeTransitions(self):
        return 'done'
        
    def executeIn(self):
        self.relocate()

class PourWilly(CyclicActionState):
    def createAction(self):
        cap=AmbiCapRed(0,Data.color)
        self.cap(cap.angle)



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