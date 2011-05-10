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
                                   transitions={'red':'Red','blue':'Blue'})
            smach.StateMachine.add('Red', Red(),
                                   transitions={'blue':'Blue','start':'endInitialisation'})
            smach.StateMachine.add('Blue', Blue(),
                                   transitions={'red':'Red','start':'endInitialisation'})
 
class Init(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['red','blue'])

    def executeTransitions(self):
       rospy.loginfo("eb beh je suis dans la transition de init")
       if Inputs.color=='red' and Inputs.start==0:
           return 'red'
       if Inputs.color=='blue'and Inputs.start==0:
           return 'blue'

class Red(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['blue','start'])

    def executeTransitions(self):
       rospy.loginfo("eb beh je suis dans la transition de red")
       if Inputs.color=='blue':
           return 'blue'
       if Inputs.start==1:
           return 'start'
       
class Blue(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['red','start'])

    def executeTransitions(self):
       rospy.loginfo("eb beh je suis dans la transition de blue")
       if Inputs.color=='red':
           return 'red'
       if Inputs.start==1:
           return 'start'