#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
from rospy import loginfo
import smach
import smach_ros
import smach_msgs

from arp_master.strat.stratVierge.util.CyclicState import CyclicState
from arp_master.strat.stratVierge.util.CyclicActionState import CyclicActionState
from arp_master.strat.stratVierge.util.Inputs import Inputs
from arp_master.strat.stratVierge.util.Data import Data
from arp_master.strat.stratVierge.util.EndMatchPreempter import EndMatchPreempter
from arp_master.strat.stratVierge.util.PreemptiveStateMachine import PreemptiveStateMachine
from arp_master.strat.stratVierge.util.PreemptiveCyclicState import PreemptiveCyclicState
from arp_master.strat.stratVierge.util.ObstaclePreempter import FrontObstaclePreempter
from arp_master.strat.stratVierge.util.ObstaclePreempter import RearObstaclePreempter
from arp_master.strat.stratVierge.util.WaiterState import WaiterState
from arp_master.strat.stratVierge.util.TableVierge import *
from arp_master.strat.stratVierge.util.UtilARD import *

from math import *




class Middlegame(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['endMiddlegame'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'endMiddlegame'})
            # other states
            PreemptiveStateMachine.add('EtatA',
                      EtatA(),
                      transitions={'succeeded':'EtatB', 'aborted':'ReverseOrder'})
            #as initial state is not the preemptive one, it is necessary to add the information here !
            self.setInitialState('EtatA')
            
            PreemptiveStateMachine.add('EtatB',
                      EtatB(),
                      transitions={'succeeded':'EtatA', 'aborted':'ReverseOrder'})

            PreemptiveStateMachine.add('ReverseOrder',
                      ReverseOrder(),
                      transitions={'succeeded':'endMiddlegame', 'aborted':'endMiddlegame'})


############### Ordres de motion

class EtatA(CyclicActionState):
    def createAction(self):
        pose = AmbiPoseRed(0.7, 0.7,0, Data.color)
        self.pointcap(pose.x, pose.y, pose.theta)

class EtatB(CyclicActionState):
    def createAction(self):
        pose = AmbiPoseRed(-0.7,-0.7,pi, Data.color)
        self.pointcap(pose.x, pose.y, pose.theta)
         
################# REVERSER
class ReverseOrder(CyclicActionState):
    def createAction(self):
        order=Data.listReplayOrders.pop() #retourne le dernier element et l'enleve de la liste
        self.executeReplayOrder(order) 
