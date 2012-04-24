#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs
import os

from arp_master.strat.util.CyclicState import CyclicState
from arp_master.strat.util.CyclicActionState import CyclicActionState
from arp_master.strat.util.Inputs import Inputs
from arp_master.strat.util.Data import Data
from arp_master.strat.util.EndMatchPreempter import EndMatchPreempter
from arp_master.strat.util.PreemptiveStateMachine import PreemptiveStateMachine
from arp_master.strat.util.PreemptiveCyclicState import PreemptiveCyclicState
from arp_master.strat.util.WaiterState import WaiterState
from arp_master.strat.util.TableVierge import *
from arp_master.strat.util.UtilARD import *

from math import *

class Endgame(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['endEndgame'])
        with self:
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(0),
                                             transitions={'endMatch':'endEndgame'})
            
            PreemptiveStateMachine.add('EndGameBip',
                      EndGameBip(),
                      transitions={'end':'endEndgame'})
            #as initial state is not the preemptive one, it is necessary to add the information here !
            self.setInitialState('EndGameBip')

            
class EndGameBip(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['end'])
    
    def executeIn(self):
        os.system("beep -f 300 -l100 -r2") 
    
    def executeTransitions(self):
            return 'end'
