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

class DropPion(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['dropped','endmatch'])
        with self:
            #preemptive states
            PreemptiveStateMachine.addPreemptive('ObstaclePreemption',
                                             ObstaclePreemption(),
                                             transitions={'avoid':'EscapeObstacle','impossible':'EscapePoint'})
            PreemptiveStateMachine.addPreemptive('EndMatchPreemtion',
                                             EndMatchPreemtion(),
                                             transitions={'endPreemption':'endmatch'})

class Drop(CyclicActionState): 
    def createAction(self):
        self.pointcap(0,0,0)
        
 
 
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