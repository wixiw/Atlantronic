#!/usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
from arp_master.util import *
from arp_master.fsmFramework import *
from EndMatchPreempter import *

# A LocalStrategicAction is the elementary strategy block representing a robot functionnality
# such as : pickObjectXXX, unloadObjectThere, doSomeShit, ...
# 
# It relies on a GlobalStrategy block to begin at its entry point. 
# When in a LocalStrategicAction is executing it manages local issues (such as actuator blocked, opponent is close, ...)
# as much as possible.
# 
# The LogicalStrategicAction is responsible for putting the robot in a standard idle state before exiting 
# (typically actuators shall not be deplyed any more)
# 
# Only 3 transitions are allowed : 
# _ succeeded (to be triggered by child classes)
# _ failed (to be triggered by child classes)
# _ almostEndGame (automatically triggered by LocalStrategicAction
#
# As this machine is automatically adding a state in its constructor you HAVE TO precise the entry state in 
# the child class after having declared it : "self.setInitialState('GoToFresco')"
#
# @param : Duration in s (time to reserve for the endOfMatch)
class LocalStrategicAction(PreemptiveStateMachine):
    def __init__(self, p_endOfMatchDelay):
        PreemptiveStateMachine.__init__(self,outcomes=['succeeded','failed','almostEndGame'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-p_endOfMatchDelay),
                                             transitions={'endMatch':'almostEndGame'})
            
    
    #This function should always be overriden : provide the entry point
    def getEntryYellowPose(self):
        return None