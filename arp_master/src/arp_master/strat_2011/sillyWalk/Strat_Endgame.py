#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs

from arp_master.strat.util.CyclicState import CyclicState
from arp_master.strat.util.CyclicActionState import CyclicActionState
from arp_master.strat.util.Inputs import Inputs
from arp_master.strat.util.Data import Data
from arp_ods.msg import OrderGoal
from arp_ods.msg import OrderAction

from math import pi

from arp_master.strat.util.Table2011 import *
from arp_master.strat.util.UtilARD import *

class Endgame(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['endEndgame'])
        with self:
            smach.StateMachine.add('EndgameState',
                      EndgameState(),
                      transitions={'ok':'endEndgame'})
      
class EndgameState(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['ok'])

    def executeTransitions(self):
        return 'ok'      
