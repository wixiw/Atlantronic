#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs

from CyclicState import CyclicState
from Inputs import Inputs
from Data import Data
from arp_ods import OrderGoal

class StartSequence(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['gogogo'])
        with self:
            smach.StateMachine.add('Recal_petit_bord',
                      SimpleActionState('MotionControl',
                                        OrderGoal,
                                        goal_cb=Recal_petit_bord_goalcb),
                      transitions={'succeeded':'Retour_petit_bord'})
            
            smach.StateMachine.add('Retour_petit_bord',
                      SimpleActionState('MotionControl',
                                        OrderGoal,
                                        goal_cb=Retour_petit_bord_goalcb),
                      transitions={'succeeded':'WaitForMatch'})

            smach.StateMachine.add('WaitForMatch', WaitForMatch(),
                                   transitions={'start':'gogogo'})
    
            
    def Recal_petit_bord_goalcb():
        goal=OrderGoal()
        goal.x_des=0
        goal.ydes=0
        goal.theta_des=0
        return goal
        
    def Retour_petit_bord_goalcb():
        goal=OrderGoal()
        goal.x_des=0
        goal.ydes=1
        goal.theta_des=0
        return goal
        
        
class WaitForMatch(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['start'])
    
    def executeTransitions(self):
       if Inputs.getstart()==1:
            return 'start'
        