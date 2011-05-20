#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs

from CyclicState import CyclicState
from CyclicActionState import CyclicActionState
from PreemptiveStateMachine import PreemptiveStateMachine
from PreemptiveCyclicState import PreemptiveCyclicState
from Inputs import Inputs
from Data import Data
from arp_ods.msg import OrderGoal
from arp_ods.msg import OrderAction

from math import pi

from Table2011 import *
from UtilARD import *

class Middlegame(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['endMiddlegame'])
        with self:
            #preemptive states
            PreemptiveStateMachine.addPreemptive('ObstaclePreemption',
                                             ObstaclePreemption(),
                                             transitions={'sortie':'CreateWalk'})
            PreemptiveStateMachine.addPreemptive('EndMatchPreemtion',
                                             EndMatchPreemtion(),
                                             transitions={'sortie':'endMiddlegame'})
            # other states
            PreemptiveStateMachine.add('CreateWalk',
                      CreateWalk(),
                      transitions={'toStep1':'Step1','toStep2':'Step2','toStep3':'Step3'})
            #as initial state is not the preemptive one, it is necessary to add the information here !
            self.setInitialState('CreateWalk')
            
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
        CyclicState.__init__(self, outcomes=['toStep1','toStep2','toStep3'])
    
    def executeTransitions(self):
        #mon role est de creer les ordres de mouvements pour un cycle de silly walk
        #je vais creer 3 cases de silly walk et envoyer dessus le robot
        
        #dirRobot=getDirection(Inputs.gettheta())
        #caseres=getClosestCaseInDir(dirRobot,Data.color)
        
        #
        
        print "x,y :%.3f , %.3f"%(Inputs.getx(),Inputs.gety())
        current_case=getCase(Inputs.getx(),Inputs.gety())
        print "cur case:"
        current_case.toText()
        
        direction=getDirection4Q(Inputs.gettheta())
        dirParcours=direction.getRotated(pi/4)
        
        if current_case.color()==Data.adv_color:
            case1=current_case
        else:
            case1=current_case.getClosestInDirection(direction.getInverse(),Data.adv_color)
        
        case2=case1.getFurthestInDirection(dirParcours,Data.adv_color)
        # des fois cases 3 n'existe pas ! ca serait en dehors de la table !!!!
        case3=case2.getClosestInDirection(dirParcours.getRotated(pi/4),Data.color)
        Data.case1=case1
        Data.case2=case2
        Data.case3=case3
        Data.dirParcours=dirParcours
        
        print "direction robot :"
        direction.toText()
        print "direction parcours:"
        dirParcours.toText()

        print "case1,2,3:"
        case1.toText()
        case2.toText()
        #case3.toText()

        
        return 'toStep1'
 
class Step1(CyclicActionState):
    def createAction(self):
        self.pointcap_reverse(Data.case1.xCenter,Data.case1.yCenter,Data.dirParcours.angle())

class Step2(CyclicActionState):
    def createAction(self):
        self.pointcap(Data.case2.xCenter,Data.case2.yCenter,Data.dirParcours.angle())

class Step3(CyclicActionState):
    def createAction(self):
        angleobj=Data.dirParcours.angle()+pi/4+pi/8
        (xobj,yobj)=Data.case3.coord_WhenPionMilieu(angleobj)
        self.pointcap(xobj,yobj,angleobj)

class ObstaclePreemption(PreemptiveCyclicState):
    def __init__(self):
        PreemptiveCyclicState.__init__(self, outcomes=['sortie'])

    def preemptionCondition(self):
        if Inputs.getobstacle()==1:
            return True
        else:
            return False
       
    def executeTransitions(self):
        return 'sortie'
    
class EndMatchPreemtion(PreemptiveCyclicState):
    def __init__(self):
        PreemptiveCyclicState.__init__(self, outcomes=['sortie'])
        self.match_duration=rospy.get_param("/match_duration")

    def preemptionCondition(self):
        if (rospy.get_rostime()-Data.start_time).to_sec()>self.match_duration:
            return True
        else:
            return False
       
    def executeTransitions(self):
        return 'sortie'