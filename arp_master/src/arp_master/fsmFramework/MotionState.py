#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs
import actionlib


from CyclicActionState import *
from arp_master.util import *
from arp_ods.msg import *
from math import *

# ** You should not have to use this state **
# Prefer the use of class from MotionStateCollection
class MotionState(CyclicActionState):
    
    def __init__(self):
        CyclicState.__init__(self,outcomes=['succeeded','timeout'])
        self.timeout = 10
        self.lastStart=None
        #maximum number of retry
        self.nMaxTry=1
        # time we will wait before try again
        self.timeoutWAIT=2

   
    def execute(self,userdata):
        self.timeIn=rospy.get_rostime().secs
        Inputs.update()
        self.actionCreated = False
        #this should always be overrided !
        self.createAction()
        if self.actionCreated==False:
            rospy.logerr("aucune Action creee dans un etat qui etait fait pour ca !")
            return
        
        #current retry number
        self.curTry=0
        #current state. can be TRY or WAIT
        self.motionState='TRY'
        
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
            self.trans=self.executeTransition()  
                     
            if self.trans!=None:
                self.executeOut()
                return self.trans
            
            Data.stateMachineRate.sleep()
            Inputs.update()
        rospy.logerr("boucle d'etat cassee par le shutdown")
        return "shutdown"
        
        
    #this is the transition function. It checks if the order has succeded or not, following action status
    # it also check if there is an obstacle
    def executeTransition(self):
        obstacle=self.isObstacle()
        state=self.client.get_state()
        #check if the timeout is fired
        if (self.timeout != 0 and rospy.get_rostime().secs-self.timeIn>self.timeout):
                return 'timeout'
         
        # are we moving ?
        if self.motionState=='TRY':    
            if state==actionlib.GoalStatus.SUCCEEDED:
                return 'succeeded'
            
            if state==actionlib.GoalStatus.ABORTED or state==actionlib.GoalStatus.REJECTED or state==actionlib.GoalStatus.LOST or state==actionlib.GoalStatus.PREEMPTED or self.isFrontObstacle() or self.isRearObstacle():
                self.client.cancel_all_goals()
                return 'timeout'
            
            #we have a problem and we will wait before try again
            if  obstacle and self.curTry<self.nMaxTry:
                self.client.cancel_all_goals()
                self.motionState='WAIT'
                self.timeInWAIT=rospy.get_rostime().secs
                rospy.logerr("MOTION STATE : try passe en wait")
                return None 
            # we have a problem and we wont try again
            if  obstacle and self.curTry>=self.nMaxTry:
                rospy.logerr("MOTION STATE : try passe en fail")
                return 'timeout'
        
        #are we waiting for the next try ?
        if self.motionState=='WAIT':
            if  rospy.get_rostime().secs-self.timeInWAIT>self.timeoutWAIT:
                rospy.logerr("MOTION STATE : wait passe en try")
                self.motionState='TRY'
                self.createAction()
                self.curTry=self.curTry+1
                return None
            else:
                return None
              
    #check for obstacles
    def isObstacle(self):
        opp = Inputs.getOpponents()
        #opp.printOpponents()
        if opp.closest_distance<0.400:
            rospy.logerr("MOTION STATE : vu qqn")
            return True
        else:
            return False
 
     