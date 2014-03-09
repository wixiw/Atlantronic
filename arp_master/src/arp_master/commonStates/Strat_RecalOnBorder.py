#!/usr/bin/env python


#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
import os
from SetPosition import *

from arp_master.strat_2014.common_2014.Table2014 import *


#
# This is the default a1 level state for any strategy. Except for testing purpose it should be overided in any strategy
#
##################################################


class RecalOnBorder(smach.StateMachine):
    def __init__(self,borderName):
        smach.StateMachine.__init__(self,outcomes=['recaled','non-recaled','problem'])
        with self:
            smach.StateMachine.add('ForwardOrder',
                                   OpenLoopOrder(vx = 0.05, 
                                           vy = 0, 
                                           vh = 0, 
                                           duration=3.0),
                                   transitions={'succeeded':'setPosition', 'timeout':'problem'}) 
            smach.StateMachine.add('setPosition',
                                   SetPositionState(*Table2014.recallWalls[borderName]),
                                   transitions={'succeeded':'recaled', 'timeout':'problem'})
          

