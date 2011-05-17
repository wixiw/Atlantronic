#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs
import actionlib

from math import cos
from math import sin
from math import pi

from CyclicState import CyclicState
from Inputs import Inputs
from Data import Data

from arp_ods.msg import OrderGoal
from arp_ods.msg import OrderAction


class CyclicActionState(CyclicState):
    
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['succeeded','aborted'])
        self.preemptiveStates=[]
        
    def execute(self,userdata):
        Inputs.update()
        self.actionCreated = False
        #this should always be overrided !
        self.createAction()
        if self.actionCreated==False:
            rospy.logerr("aucune Action creee dans un etat qui etait fait pour ca !")
            return
        
        while(not rospy.is_shutdown()):
            
            self.executeWhile()
            
             #check if preemption transitions
            for p in self.preemptiveStates:
                label=p[0]
                state=p[1]
                preempted=state.preemptionCondition()
                if preempted:
                    return label
                
            #is the order terminated ?
            trans=self.executeClientTransition()  
                     
            if trans!=None:
                self.executeOut()
                return trans
            
            Data.stateMachineRate.sleep()
            Inputs.update()
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
        goal.move_type=move_type
        goal.reverse=reverse
        
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
        self.createMotionControlAction(0,0,theta,'CAP',False)
  
    def forward(self,dist):
        self.createMotionControlAction(Inputs.getx()+dist*cos(Inputs.gettheta()),Inputs.gety()+dist*sin(Inputs.gettheta()),Inputs.gettheta(),'POINTCAP',False)
   
    def backward(self,dist):
        self.createMotionControlAction(Inputs.getx()+dist*cos(Inputs.gettheta()+pi),Inputs.gety()+dist*sin(Inputs.gettheta()+pi),Inputs.gettheta(),'POINTCAP',True)




    
    
    