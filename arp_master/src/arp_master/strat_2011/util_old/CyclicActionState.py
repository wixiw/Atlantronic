#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs
import actionlib

from math import *

from CyclicState import CyclicState
from Inputs import Inputs
from Data import Data

from arp_ods.msg import OrderGoal
from arp_ods.msg import OrderAction

from Table2011 import *
from UtilARD import *

from RewindOrder import RewindOrder

class CyclicActionState(CyclicState):
    
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['succeeded','aborted'])
        self.preemptiveStates=[]
        self.lastStart=None
        self.initClients()
        self.blinding_period=rospy.get_param("/blinding_period")
        
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
                    self.client.cancel_all_goals()
                    return label
                
            #is the order terminated ?
            self.trans=self.executeClientTransition()  
                     
            if self.trans!=None:
                self.executeOut()
                return self.trans
            
            Data.stateMachineRate.sleep()
            Inputs.update()
        rospy.logerr("boucle d'etat cassee par le shutdown")

    def executeClientTransition(self):
        state=self.client.get_state()
        if state==actionlib.GoalStatus.SUCCEEDED:
            return 'succeeded'
        
        if state==actionlib.GoalStatus.ABORTED or state==actionlib.GoalStatus.REJECTED or state==actionlib.GoalStatus.LOST or state==actionlib.GoalStatus.PREEMPTED:
            self.client.cancel_all_goals()
            return 'aborted'  
        
        #all others are considered "waiting"
        #Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST
       
             
    # generic motioncontrol action creator.
    def createMotionControlAction(self,x,y,theta,move_type,reverse,passe):
        self.client = actionlib.SimpleActionClient('MotionControl', OrderAction)
        goal=OrderGoal()
        goal.x_des=x
        goal.y_des=y
        goal.theta_des=theta
        goal.move_type=move_type
        goal.reverse=reverse
        goal.passe=passe
        
        #si le mouvement est suffisant, on lance le relocalisateur
        #if sqrt((x-Inputs.getx())**2+(y-Inputs.gety())**2)>0.100 and Data.allowRelocate==True:
            #rospy.loginfo("RELOC>> trying to relocate because the movement asked is big enough to handle a jump")
            #self.relocate()
            ##########################FONCTION BLOQUANTE !
            #self.waitForStart()
        
        self.registerRewindOrder(x,y,theta,move_type,reverse,passe)
            
        # THIS IS BLOCKING ! <<<<<<<<<<<<<<<<<<<<<<< !!!!!!!!!!!!!!
        self.client.wait_for_server()
        self.client.cancel_all_goals
        self.client.send_goal(goal)
        
        self.actionCreated=True
        
    # these are useful function that allow not to give all parameters
    def pointcap(self,x,y,theta):
        self.createMotionControlAction(x,y,theta,'POINTCAP',False,False)
      
    def pointcap_pass(self,x,y,theta):
        self.createMotionControlAction(x,y,theta,'POINTCAP',False,True)
   
        
    def pointcap_reverse(self,x,y,theta):
        self.createMotionControlAction(x,y,theta,'POINTCAP',True,False)
    
    def cap(self,theta):
        self.createMotionControlAction(0,0,theta,'CAP',False,False)
  
    def forward(self,dist):
        self.createMotionControlAction(Inputs.getx()+dist*cos(Inputs.gettheta()),Inputs.gety()+dist*sin(Inputs.gettheta()),Inputs.gettheta(),'POINTCAP',False,False)
   
    def backward(self,dist):
        self.createMotionControlAction(Inputs.getx()+dist*cos(Inputs.gettheta()+pi),Inputs.gety()+dist*sin(Inputs.gettheta()+pi),Inputs.gettheta(),'POINTCAP',True,False)


    def dropOnCase(self,case):
        res=calculateRobotForDrop(Inputs.getx(),Inputs.gety(),case.xCenter,case.yCenter,Robot.xPion,Robot.yPion)
        if res==None:
            return False
        else:
            (x,y,cap)=res
            self.createMotionControlAction(x,y,cap,'POINTCAP',False,False)
            return True
        
    def dropOnCaseCap(self,case,cap):
        (xrobot,yrobot)=case.coord_WhenPionMilieu(cap)
        self.pointcap(xrobot,yrobot, cap )


    def registerRewindOrder(self,x,y,theta,move_type,reverse,passe):
        Data.listRewindOrders.append(RewindOrder(x,y,theta,move_type,reverse,passe))   
    
    def executeRewindOrder(self,RewindOrder):
        self.createMotionControlAction(RewindOrder.reversegoal.x_des,RewindOrder.reversegoal.y_des,RewindOrder.reversegoal.theta_des,RewindOrder.reversegoal.move_type,RewindOrder.reversegoal.reverse,RewindOrder.reversegoal.passe)
   
    
    