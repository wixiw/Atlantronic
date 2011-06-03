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
from arp_master.strat.util.EndMatchPreempter import EndMatchPreempter
from arp_master.strat.util.WaiterState import WaiterState
from arp_master.strat.util.Inputs import Inputs
from arp_master.strat.util.Data import Data
from arp_ods.msg import OrderGoal
from arp_ods.msg import OrderAction

import SubStrat_GetPionBord
import SubStrat_DropPion

from math import pi

from arp_master.strat.util.Table2011 import *
from arp_master.strat.util.UtilARD import *

class Middlegame_D(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['endMiddlegame'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-5.0),
                                             transitions={'endMatch':'endMiddlegame'})
            # other states
            PreemptiveStateMachine.add('Selector',
                      Selector(),
                      transitions={'getpion':'GetPionBord'})
            #as initial state is not the preemptive one, it is necessary to add the information here !
            self.setInitialState('Selector')
            
            PreemptiveStateMachine.add('GetPionBord', SubStrat_GetPionBord.GetPionBord(),
                                   transitions={'got':'DropPion','obstacle':'Selector','endmatch':'endMiddlegame','problem':'Selector'})
            PreemptiveStateMachine.add('DropPion', SubStrat_DropPion.DropPion(),
                                   transitions={'dropped':'Selector','obstacle':'Selector','endmatch':'endMiddlegame'})


#l'etat qui decide de ce qui va etre fait maintenant
class Selector(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['getpion'])
    

    def executeTransitions(self):
        rospy.loginfo("Entering selector")
        rospy.loginfo(str(Data.listStatusPionsBord))
        #premier tour: je regarde si roi ou reine
        for i in range(4):
            if Data.listStatusPionsBord[i]=='FIGURE':
                Data.pionBordObjectif=PionBord(i,Data.color)
                Data.listStatusPionsBord[i]='DONE'
                rospy.loginfo("Going for a figure in %i"%i)
                return 'getpion'
        #deuxieme tour: premier endroit dispo
        for i in range(4):
            if Data.listStatusPionsBord[i]=='NORMAL':
                Data.pionBordObjectif=PionBord(i,Data.color)
                Data.listStatusPionsBord[i]='DONE'
                rospy.loginfo("Going for a pion in %i"%i)
                return 'getpion'
        
        #si je suis arrive la : plus peronne de prenable
                
        rospy.loginfo("no normal pion and no figure")
        if Data.pionBordObjectif==None:
            Data.pionBordObjectif=PionBord(0,Data.color)
        else:
            Data.pionBordObjectif=PionBord((Data.pionBordObjectif.rang+1)%4,Data.color)
            
        return 'getpion'
        

