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

class Middlegame_C(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['endMiddlegame'])
        with self:
            #preemptive states
            PreemptiveStateMachine.addPreemptive('ObstaclePreemption',
                                             ObstaclePreemption(),
                                             transitions={'avoid':'EscapeObstacle','impossible':'EscapePoint'})
            PreemptiveStateMachine.addPreemptive('EndMatchPreemtion',
                                             EndMatchPreemtion(),
                                             transitions={'endPreemption':'endMiddlegame'})
            # other states
            PreemptiveStateMachine.add('CreateWalk',
                      CreateWalk(),
                      transitions={'toStep1':'Step1','toStep2':'Step2','toStep3':'Step3','impossible':'EscapePoint'})
            #as initial state is not the preemptive one, it is necessary to add the information here !
            self.setInitialState('CreateWalk')
            
            PreemptiveStateMachine.add('EscapeObstacle',
                      EscapeObstacle(),
                      transitions={'succeeded':'CreateWalk','aborted':'CreateWalk'})
           
           
            PreemptiveStateMachine.add('EscapePoint',
                      EscapePoint(),
                      transitions={'succeeded':'CreateWalk','aborted':'CreateWalk'})
            
            
            PreemptiveStateMachine.add('Step1',
                      Step1(),
                      transitions={'succeeded':'Step2','aborted':'CreateWalk'})
            PreemptiveStateMachine.add('Step2',
                      Step2(),
                      transitions={'succeeded':'Step3','aborted':'CreateWalk'})
            PreemptiveStateMachine.add('Step3',
                      Step3(),
                      transitions={'succeeded':'CreateWalk','aborted':'CreateWalk'})

#createWalk est le dispatcher
#step1 est le recul sur la case adverse
#step2 est l'avancee en diagonale sur les cases adverses
#step3 est la poussee du palet sur sa propre case

class CreateWalk(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['toStep1','toStep2','toStep3','impossible'])
    
    def executeTransitions(self):
        #mon role est de creer les ordres de mouvements pour un cycle de silly walk
        #je vais creer 3 cases de silly walk et envoyer dessus le robot

        case1=None
        case2A=None
        case2B=None
        case3A=None
        case3B=None

        current_case=getCase(Inputs.getx(),Inputs.gety())

        direction=getDirection8Q(Inputs.gettheta())
        
        if direction.isOrtho():
            dirParcours=direction.getRotated(pi/4)
        else:
            dirParcours=direction
        
        if current_case.color()==Data.adv_color:
            case1=current_case
        else:
            case1=current_case.getClosestInDirection(direction.getInverse(),Data.adv_color)     
        
        #case 2A: la plus loin en diagonale
        if case1!=None:
            case2A=case1.getFurthestInDirection(dirParcours,Data.adv_color)

        #case 2B: juste une case avant la 2A dans la diagonale
        if case2A!=None:
            case2B=case2A.getClosestInDirection(dirParcours.getRotated(pi),Data.adv_color)
            if case2B==case1:
                case2B=None
        
        #des fois elle est en dehors de la table la case 3A. d'ou le 2B et 3B
        if case2A!=None:
            case3A=case2A.getClosestInDirection(dirParcours.getRotated(pi/4),Data.color)
        if case2B!=None:
            case3B=case2B.getClosestInDirection(dirParcours.getRotated(pi/4),Data.color)
        
        #c'est la que je regarde si je fais le cas "standard" ou "je m'arrete une case avant"
        if case2A==None or case3A==None or ((case2A.i,case2A.j) in Data.liste_cases2_faites):
            case2=case2B
            case3=case3B
        else:
            case2=case2A
            case3=case3A
        
        loginfo("----- Calcul de SillyWalk ----")
        loginfo(">> initialement: ")
        loginfo("ma case: "+str(current_case))
        loginfo("ma direction: "+str(direction))
        loginfo(">> calculs: ")
        loginfo("case1: "+str(case1))
        loginfo("case2A: "+str(case2A))
        loginfo("case2B: "+str(case2B))
        loginfo("case3A: "+str(case3A))
        loginfo("case3B: "+str(case3B))
        loginfo(">> conclusion: ")
        loginfo("case1: "+str(case1))
        loginfo("case2: "+str(case2))
        loginfo("case3: "+str(case3))
        loginfo("dirParcours: "+str(dirParcours))
        loginfo("----- Fin du calcul ----")
        
        #des fois je trouve pas de solution !
        if case1==None or case2==None or case3==None:
            return 'impossible'
        
        #je met ca dans Data car les etats suivant vont l'utiliser
        Data.case1=case1
        Data.case2=case2
        Data.case3=case3
        Data.dirParcours=dirParcours

        # j'ajoute dans la liste des case2 que j'ai parcourues
        Data.liste_cases2_faites.append((case2.i,case2.j))
        
        return 'toStep1'
        
    
class EscapePoint(CyclicActionState):
    def createAction(self):
        #normalement on vient jamais ici, c'est la voiture balai... alors bon on sort un point au hasard, quoi
        self.pointcap((random()*2.0-1.0)*0.6,random()*0.6,(random()*2.0-1.0)*pi)
 
class Step1(CyclicActionState):
    def createAction(self):
        #j'y vais en droit ou reverse, suivant le chemin le plus court en fait
        xobj=Data.case1.xCenter
        yobj=Data.case1.yCenter
        capdirect=atan2(yobj-Inputs.gety(),xobj-Inputs.getx())
        if abs(normalizeAngle(capdirect-Inputs.gettheta()))<pi/2:
            self.pointcap(xobj,yobj,Data.dirParcours.angle())
        else:
            self.pointcap_reverse(xobj,yobj,Data.dirParcours.angle())

class Step2(CyclicActionState):
    def createAction(self):
        dx=-0.100*cos(Data.dirParcours.angle())
        dy=-0.100*sin(Data.dirParcours.angle())
        self.pointcap(Data.case2.xCenter+dx,Data.case2.yCenter+dy,Data.dirParcours.angle())

class Step3(CyclicActionState):
    def createAction(self):
        self.dropOnCase(Data.case3)

class ObstaclePreemption(PreemptiveCyclicState):
    def __init__(self):
        PreemptiveCyclicState.__init__(self, outcomes=['avoid','impossible'])
        self.blinding_period=rospy.get_param("/blinding_period")

    def preemptionCondition(self):
        if Inputs.getobstacle()==1 and rospy.get_rostime().secs-Data.time_obstacle>self.blinding_period:
            Data.time_obstacle=rospy.get_rostime().secs
            return True
        else:
            return False
       
    def executeTransitions(self):
        direction=getDirection4Q(Inputs.gettheta())
        current_case=getCase(Inputs.getx(),Inputs.gety())
        Data.caseEscape=current_case.getFurthestInDirection(direction.getRotated(pi),Data.adv_color)
        if Data.caseEscape==None:
            return 'impossible'
        return 'avoid'
    
    
class EscapeObstacle(CyclicActionState):
    def createAction(self):
        self.pointcap_reverse(Data.caseEscape.xCenter,Data.caseEscape.yCenter,Data.caseEscape.dirForSillyWalk().angle())
    
    
class EndMatchPreemtion(PreemptiveCyclicState):
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
    
    
class ObstacleARPreemption(PreemptiveCyclicState):
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