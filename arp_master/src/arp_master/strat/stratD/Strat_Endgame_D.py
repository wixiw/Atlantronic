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
from arp_master.strat.util.ObstaclePreempter import FrontObstaclePreempter
from arp_master.strat.util.ObstaclePreempter import RearObstaclePreempter
from arp_master.strat.util.EndMatchPreempter import EndMatchPreempter
from arp_master.strat.util.WaiterState import WaiterState
from arp_master.strat.util.Inputs import Inputs
from arp_master.strat.util.Data import Data
from arp_ods.msg import OrderGoal
from arp_ods.msg import OrderAction

from math import *

from arp_master.strat.util.Table2011 import *
from arp_master.strat.util.UtilARD import *

class Endgame_D(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['endEndgame'])
        with self:
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(0),
                                             transitions={'endMatch':'endEndgame'})
            
            PreemptiveStateMachine.add('FinalDrop',
                      FinalDrop(),
                      transitions={'succeeded':'endEndgame','aborted':'FinalDrop'})
            self.setInitialState('FinalDrop')

class FinalDrop(CyclicActionState):
    def createAction(self):
        maCase=getCase(Inputs.getx(),Inputs.gety())
        direction=getDirection4Q(Inputs.gettheta())
        casepotential1=None
        casepotential2=None
        casepotential3=None
        
        if maCase.color()==Data.color:
            #si c'est de ma couleur, alors je vais sur une case en diagonale devant moi
            casepotential1=maCase.getClosestInDirection(direction.getRotated(pi/4),Data.color )
            casepotential2=maCase.getClosestInDirection(direction.getRotated(-pi/4),Data.color )
        else:
            # si c'est celle de l'adversairen alors je chope une case a moi devant a droite ou a gauche
            casepotential1=maCase.getClosestInDirection(direction,Data.color )
            casepotential2=maCase.getClosestInDirection(direction.getRotated(pi/2),Data.color )
            casepotential3=maCase.getClosestInDirection(direction.getRotated(-pi/2),Data.color )
            
        if casepotential1!=None and casepotential1.inTable():
            self.dropOnCase(casepotential1)
            return
        if casepotential2!=None and casepotential2.inTable():
            self.dropOnCase(casepotential2)
            return
        if casepotential3!=None and casepotential3.inTable():
            self.dropOnCase(casepotential3)
            return
        
        self.cap(Inputs.gettheta())  
            