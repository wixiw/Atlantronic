#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs
import CyclicState


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
 
class Init(CyclicState.CyclicState):
    def __init__(self):
        CyclicState.CyclicState.__init__(self, outcomes=['red','blue'])

    def executeTransition(self):
       global color
       rospy.loginfo("eb beh je suis dans la transition de init")
       if color=='red' and start==0:
           return 'red'
       if color=='blue'and start==0:
           return 'blue'

class Red(CyclicState.CyclicState):
    def __init__(self):
        CyclicState.CyclicState.__init__(self, outcomes=['blue','start'])

    def executeTransition(self):
       global color
       rospy.loginfo("eb beh je suis dans la transition de red")
       if color=='blue':
           return 'blue'
       if start==1:
           return 'start'
       
class Blue(CyclicState.CyclicState):
    def __init__(self):
        CyclicState.CyclicState.__init__(self, outcomes=['red','start'])

    def executeTransition(self):
       global color
       rospy.loginfo("eb beh je suis dans la transition de blue")
       if color=='red':
           return 'red'
       if start==1:
           return 'start'