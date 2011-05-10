#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs

from CyclicState import CyclicState
from Inputs import Inputs


class Initialisation(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['endInitialisation'])
        with self:
            smach.StateMachine.add('Init', Init(),
                                   transitions={'start':'endInitialisation'})

 
class Init(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['start'])

    def executeTransitions(self):
       rospy.loginfo("eb beh je suis dans la transition de init")
       rospy.loginfo("color="+Inputs.color)
       if Inputs.start==1:
           return 'start'

    def executeOut(self):
        rospy.loginfo("color="+Inputs.color)