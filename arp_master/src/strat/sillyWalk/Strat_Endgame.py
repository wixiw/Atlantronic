#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs

from CyclicState import CyclicState
from CyclicActionState import CyclicActionState
from Inputs import Inputs
from Data import Data
from arp_ods.msg import OrderGoal
from arp_ods.msg import OrderAction

from math import pi

from Table2011 import *
from UtilARD import *

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

    def executeOut(self):
        self.disableDrive()
        
    def executeTransitions(self):
        return 'ok'      
