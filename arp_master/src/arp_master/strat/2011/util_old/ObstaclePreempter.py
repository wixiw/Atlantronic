#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs

from Inputs import Inputs
from Data import Data

from arp_master.strat.util.PreemptiveCyclicState import PreemptiveCyclicState

class FrontObstaclePreempter(PreemptiveCyclicState):
    def __init__(self,outcomes):
        PreemptiveCyclicState.__init__(self, outcomes)
        self.blinding_period=rospy.get_param("/blinding_period")

    def preemptionCondition(self):  
        if Inputs.getObstacle()==1 and rospy.get_rostime().secs-Data.timeObstacle>self.blinding_period and Data.obstacleAvoidType!='None' and Inputs.getLinearVelocity()>0.010:
            Data.timeObstacle=rospy.get_rostime().secs
            return True
        else:
            return False

class RearObstaclePreempter(PreemptiveCyclicState):
    def __init__(self,outcomes):
        PreemptiveCyclicState.__init__(self, outcomes)
        self.blinding_period=rospy.get_param("/blinding_period")

    def preemptionCondition(self):  
        if Inputs.getRearObstacle()==1 and rospy.get_rostime().secs-Data.timeRearObstacle>self.blinding_period and Data.rearObstacleAvoidType!='None' and Inputs.getLinearVelocity()<-0.010:
            Data.timeRearObstacle=rospy.get_rostime().secs
            return True
        else:
            return False     
