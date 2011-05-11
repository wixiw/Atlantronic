#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
from smach_ros import SimpleActionState
import smach_msgs

from CyclicState import CyclicState
from Inputs import Inputs
from Data import Data
from arp_ods.msg import OrderGoal
from arp_ods.msg import OrderAction


class StartSequence(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['gogogo','problem'])
        with self:
            smach.StateMachine.add('Recal_petit_bord',
                      SimpleActionState('MotionControl',
                                        OrderAction,
                                        goal_cb=Recal_petit_bord_goalcb),
                      transitions={'succeeded':'Retour_petit_bord','preempted':'problem','aborted':'problem'})
            
            smach.StateMachine.add('Retour_petit_bord',
                      SimpleActionState('MotionControl',
                                        OrderAction,
                                        goal_cb=Retour_petit_bord_goalcb),
                      transitions={'succeeded':'WaitForMatch','preempted':'problem','aborted':'problem'})

            smach.StateMachine.add('WaitForMatch', WaitForMatch(),
                                   transitions={'start':'gogogo'})
    
            
def Recal_petit_bord_goalcb(userdata, goal):
    goal=OrderGoal()
    goal.x_des=0.000
    goal.y_des=0.000
    goal.theta_des=0.000
    goal.move_type='POINTCAP'
    goal.reverse=True
    return goal
        
def Retour_petit_bord_goalcb(userdata, goal):
    goal=OrderGoal()
    goal.x_des=0.000
    goal.y_des=1.000
    goal.theta_des=0.000
    goal.move_type='POINTCAP'
    goal.reverse=False
    return goal
        
        
class WaitForMatch(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['start'])
    
    def executeTransitions(self):
       if Inputs.getstart()==1:
            return 'start'
        