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
from arp_master.strat.util.Inputs import Inputs
from arp_master.strat.util.Data import Data
from arp_ods.msg import OrderGoal
from arp_ods.msg import OrderAction

from math import pi

from arp_master.strat.util.Table2011 import *
from arp_master.strat.util.UtilARD import *

class DropPion(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['dropped','endmatch'])
        with self:
            #preemptive states
            PreemptiveStateMachine.addPreemptive('ObstaclePreemption',
                                             ObstaclePreemption(),
                                             transitions={'gotonextcase':'Drop1'})
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreemption(),
                                             transitions={'endPreemption':'endmatch'})
            
            PreemptiveStateMachine.add('Drop1',
                      Drop1(),
                      transitions={'succeeded':'Drop2', 'aborted':'dropped'})
            
            self.setInitialState('Drop1')
            
            PreemptiveStateMachine.add('Drop2',
                      Drop2(),
                      transitions={'succeeded':'dropped', 'aborted':'dropped'})
            
            
            
#classe qui sert a symboliser une case de depose: on lui rajoute une occupation 
class DropCase(Case):
    def __init__(self,i_hor,j_vert):
        self.i=i_hor
        self.j=j_vert
        self.occupied=False
    
    def getCase(self):
        return AmbiCaseRed(self.i,self.j,Data.color)
        

class Drop1(CyclicActionState): 
    def __init__(self):
        CyclicActionState.__init__(self)
        # la liste des cases par ordre de priorite
        #c'est une liste de case cote rouge !
        #comme c'est cree a l'init je ne sais pas encore la couleur
        self.casePriority=[DropCase(3,3),DropCase(-1,3),DropCase(-5,3),DropCase(3,-1),DropCase(1,1),DropCase(-1,-1),DropCase(-3,1)]

    def createAction(self):
        #je parcours toutes les cases et je prend la premiere non occupee
        for dropcase in self.casePriority:
            if dropcase.occupied==False:
                #je cree la bonne case bleue ou rouge, je stocke dans Data pour les autres etats
                Data.currentDropCase=dropcase
                #je vais dessus
                self.dropOnCase(dropcase.getCase())
                return
        #si j'ai fini la liste, alors je prend la premiere (devrait pas arriver)
        self.dropOnCase(self.casePriority[0].getCase())
            
    def executeOut(self):
        #si j'ai reussi a terminer mon ordre je note que la case est occupee
        if self.trans=='succeeded':
            Data.currentDropCase.occupied=True
        
        
class Drop2(CyclicActionState):
    def createAction(self):
        self.backward(0.35)    
 
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
    
    
class ObstaclePreemption(PreemptiveCyclicState):
    def __init__(self):
        PreemptiveCyclicState.__init__(self, outcomes=['gotonextcase'])
        self.blinding_period=rospy.get_param("/blinding_period")

    def preemptionCondition(self):
        if Inputs.getobstacle()==1 and rospy.get_rostime().secs-Data.time_obstacle>self.blinding_period:
            Data.time_obstacle=rospy.get_rostime().secs
            return True
        else:
            return False
       
    def executeTransitions(self):
        Data.currentDropCase.occupied=True
        return 'gotonextcase'