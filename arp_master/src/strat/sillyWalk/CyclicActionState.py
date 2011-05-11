#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs
import actionlib

from CyclicState import CyclicState
from Inputs import Inputs
from Data import Data

from arp_ods.msg import OrderGoal
from arp_ods.msg import OrderAction

class CyclicActionState(CyclicState):
    
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['succeeded','aborted'])
        
    def execute(self,userdata):
        self.actionCreated = False
        #this should always be overrided !
        self.createAction()
        if self.actionCreated==False:
            rospy.logerr("aucune Action creee dans un etat qui etait fait pour ca !")
            return
        
        while(not rospy.is_shutdown()):
            Inputs.update()
            self.executeWhile()
            
            #is the order terminated ?
            trans=self.executeClientTransition()  
                     
            if trans!=None:
                self.executeOut()
                return trans
            
            Data.stateMachineRate.sleep()
        rospy.logerr("boucle d'etat cassee par le shutdown")

    def executeClientTransition(self):
        state=self.client.get_state()
        if state==actionlib.GoalStatus.SUCCEEDED:
            return 'succeeded'
        
        #condition non testee <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< !!!!!!!!!
        if state==actionlib.GoalStatus.ABORTED or state==actionlib.GoalStatus.REJECTED or state==actionlib.GoalStatus.LOST or state==actionlib.GoalStatus.PREEMPTED :
            return 'aborted'  
        
        #all others are considered "waiting"
        #Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST
        
             
    # generic motioncontrol action creator.
    def createMotionControlAction(self,x,y,theta,move_type,reverse):
        self.client = actionlib.SimpleActionClient('MotionControl', OrderAction)
        goal=OrderGoal()
        goal.x_des=x
        goal.y_des=y
        goal.theta_des=theta
        goal.move_type='POINTCAP'
        goal.reverse=True
        
        # THIS IS BLOCKING ! <<<<<<<<<<<<<<<<<<<<<<< !!!!!!!!!!!!!!
        self.client.wait_for_server()
        self.client.cancel_all_goals
        self.client.send_goal(goal)
        
        self.actionCreated=True
        
    # these are useful function that allow not to give all parameters
    def pointcap(self,x,y,theta):
        self.createMotionControlAction(x,y,theta,'POINTCAP',False)
        
    def pointcap_reverse(self,x,y,theta):
        self.createMotionControlAction(x,y,theta,'POINTCAP',True)
        
    def cap(self,theta):
        self.createMotionControlAction(0,0,theta,'CAP',True)
  
#    def forward(self,dist):
#        
#    
#    def backward(self,dist):
#
#    def turn(self,angle):


    
    
    