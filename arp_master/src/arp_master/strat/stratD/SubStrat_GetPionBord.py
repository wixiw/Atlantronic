#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
from rospy import loginfo
import smach
import smach_ros
import smach_msgs

from random import *

from arp_master.strat.util.CyclicState import CyclicState
from arp_master.strat.util.CyclicActionState import CyclicActionState
from arp_master.strat.util.PreemptiveStateMachine import PreemptiveStateMachine
from arp_master.strat.util.PreemptiveCyclicState import PreemptiveCyclicState
from arp_master.strat.util.ObstaclePreempter import FrontObstaclePreempter
from arp_master.strat.util.ObstaclePreempter import RearObstaclePreempter
from arp_master.strat.util.Inputs import Inputs
from arp_master.strat.util.Data import Data
from arp_ods.msg import OrderGoal
from arp_ods.msg import OrderAction

from math import pi

from arp_master.strat.util.Table2011 import *
from arp_master.strat.util.UtilARD import *

class GetPionBord(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['got','obstacle','endmatch','problem'])
        with self:
            #preemptive states
            PreemptiveStateMachine.addPreemptive('ObstaclePreemption',
                                             ObstaclePreemption(),
                                             transitions={'obstaclepreemption':'obstacle'})
            PreemptiveStateMachine.addPreemptive('RearObstaclePreemption',
                                             RearObstaclePreemption(),
                                             transitions={'rearobstaclepreemption':'obstacle'})
            
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreemption(),
                                             transitions={'endPreemption':'endmatch'})
            
            PreemptiveStateMachine.add('GotoFacePion1',
                      GotoFacePion1(),
                      transitions={'succeeded':'GotoFacePion2', 'aborted':'problem'})
            
            self.setInitialState('GotoFacePion1')
            
            PreemptiveStateMachine.add('GotoFacePion2',
                      GotoFacePion2(),
                      transitions={'succeeded':'Turn', 'aborted':'problem'})
            
            PreemptiveStateMachine.add('Turn',
                      Turn(),
                      transitions={'succeeded':'Avance', 'aborted':'problem'})

            PreemptiveStateMachine.add('Avance',
                      Avance(),
                      transitions={'succeeded':'got', 'aborted':'problem'})

class GotoFacePion1(CyclicActionState):
    def createAction(self):
        if Data.pionBordObjectif==None:
            Data.pionBordObjectif=PionBord(0,Data.color)
        cap=AmbiCapRed(-pi,Data.color)
        (xRobot,yRobot)=Data.pionBordObjectif.coord_WhenPionMilieu(cap.angle)
        self.pointcap(xRobot-0.35*cos(cap.angle),yRobot-0.35*sin(cap.angle),cap.angle)     
        
class GotoFacePion2(CyclicActionState):
    def createAction(self):
        if Data.pionBordObjectif==None:
            Data.pionBordObjectif=PionBord(0,Data.color)
        cap=AmbiCapRed(-pi,Data.color)
        (xRobot,yRobot)=Data.pionBordObjectif.coord_WhenPionMilieu(cap.angle)
        self.pointcap(xRobot,yRobot,cap.angle)    

class Turn(CyclicActionState):
    def createAction(self):
        self.cap(normalizeAngle(Inputs.gettheta()-9*pi/10))
 
class Avance(CyclicActionState):
    def createAction(self):
        self.forward(0.100) 
 
class EndMatchPreemption(PreemptiveCyclicState):
    def __init__(self):
        PreemptiveCyclicState.__init__(self, outcomes=['endPreemption'])
        self.match_duration=rospy.get_param("/match_duration")

    def preemptionCondition(self):
        if (rospy.get_rostime()-Data.start_time).to_sec()>self.match_duration:
            return True
        else:
            return False
       
    def executeTransitions(self):
        return 'endPreemption'
    
    
class ObstaclePreemption(FrontObstaclePreempter):
    def __init__(self):
        FrontObstaclePreempter.__init__(self, outcomes=['obstaclepreemption'])
       
    def executeTransitions(self):
        return 'obstaclepreemption'
    
class RearObstaclePreemption(RearObstaclePreempter):
    def __init__(self):
        RearObstaclePreempter.__init__(self, outcomes=['rearobstaclepreemption'])
       
    def executeTransitions(self):
        return 'rearobstaclepreemption'