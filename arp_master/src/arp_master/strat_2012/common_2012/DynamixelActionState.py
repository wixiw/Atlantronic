#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs
import actionlib

from arp_master.fsmFramework.CyclicState import CyclicState
from arp_master.util import *
from arp_hml.msg import *


class DynamixelActionState(CyclicState):
    
    def __init__(self):
        CyclicState.__init__(self,outcomes=['succeeded','timeout'])
        self.timeout = 5
   
    
    # the main execute function
    #it overrides the one of CyclicState
    #here, transitions are handled automatically (succeded or aborted following action result)   
    def execute(self,userdata):
        self.timeIn=rospy.get_rostime().secs
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
                
            #check if the timeout is fired
            if (self.timeout != 0 and rospy.get_rostime().secs-self.timeIn>self.timeout):
                return 'timeout'
            #is the order terminated ?
            self.trans=self.executeClientTransition()  
                     
            if self.trans!=None:
                self.executeOut()
                return self.trans
            
            Data.stateMachineRate.sleep()
            Inputs.update()
        rospy.logerr("boucle d'etat cassee par le shutdown")
        return "shutdown"
        
        
    #this is the transition function. It checks if the order has succeded or not, following action status
    # it also check if there is an obstacle
    def executeClientTransition(self):
        state=self.client.get_state()
        if state==actionlib.GoalStatus.SUCCEEDED:
            return 'succeeded'
        
        if state==actionlib.GoalStatus.ABORTED \
            or state==actionlib.GoalStatus.REJECTED \
            or state==actionlib.GoalStatus.LOST \
            or state==actionlib.GoalStatus.PREEMPTED:
            self.client.cancel_all_goals()
            return 'timeout'  
        #all others are considered "waiting"
        #Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST
    
 
     # motioncontrol action creator with a control point
    def createDynamixelAction(self,left_finger,right_finger,left_claw,right_claw):
        self.client = actionlib.SimpleActionClient('move_claw', ClawOrderAction)
        goal=ClawOrderGoal(left_finger,right_finger,left_claw,right_claw)
        
        # THIS IS BLOCKING ! <<<<<<<<<<<<<<<<<<<<<<< !!!!!!!!!!!!!!
        self.client.wait_for_server(rospy.Duration.from_sec(5.0))
        self.client.cancel_all_goals
        self.client.send_goal(goal)
        
        self.actionCreated=True       
        
        
    # these are useful motions functions that allow not to give all parameters  
    def leftFinger(self,pos):
        self.createDynamixelAction(pos,0.0,0.0,0.0)
        
    def rightFinger(self,pos):
        self.createDynamixelAction(0.0,pos,0.0,0.0)
        
    def leftClaw(self,pos):
        self.createDynamixelAction(0.0,0.0,pos,0.0)
        
    def rightClaw(self,pos):
        self.createDynamixelAction(0.0,0.0,0.0,pos)
    
    def fingers(self,left,right):
        self.createDynamixelAction(left,right,0.0,0.0)       
    
    def claws(self,left,right):
        self.createDynamixelAction(0,0,left,right)
        
        
# Use this Order to fully control the dynamixels. It reverse sides depending on the match color
# Ex : AmbiClawFingerOrder( left_finger=0, right_finger=0, left_claw=0, right_claw=0)
class AmbiClawFingerOrder(DynamixelActionState):
    def __init__(self,left_finger, right_finger, left_claw, right_claw):
        DynamixelActionState.__init__(self)
        self.left_finger = left_finger
        self.right_finger = right_finger
        self.left_claw = left_claw
        self.right_claw = right_claw
        
    def createAction(self):
        if Data.color is "red":
            self.createDynamixelAction(self.left_finger,self.right_finger,
                                   self.left_claw, self.right_claw)
        else:
            self.createDynamixelAction(self.right_finger,self.left_finger,
                                   self.right_claw, self.left_claw)   
            

# You should only use this state when you have to do some color dependent work (else prefer the AmbiClawOrder)
class ClawFingerOrder(DynamixelActionState):
    def __init__(self,left_finger, right_finger, left_claw, right_claw):
        DynamixelActionState.__init__(self)
        self.left_finger = left_finger
        self.right_finger = right_finger
        self.left_claw = left_claw
        self.right_claw = right_claw
        
    def createAction(self):
        self.createDynamixelAction(self.left_finger,self.right_finger,
                                   self.left_claw, self.right_claw)
    
# Use this state to set fingers pre created states such as : open, close ...  
# for any of those states the claws will be maintained closed (so take care if they are already opened)
class FingersOnlyState(AmbiClawFingerOrder):
    def __init__(self,state):
        if state is 'open':
            AmbiClawFingerOrder.__init__(self,0.5,0.5,-1.8,-1.8)
        elif state is 'close':            
            AmbiClawFingerOrder.__init__(self,-1.8,-1.8,-1.8,-1.8)
        elif state is 'open_left':
            AmbiClawFingerOrder.__init__(self,0.5,0.0,-1.8,-1.8)
        elif state is 'open_right':
            AmbiClawFingerOrder.__init__(self,0.0,0.5,-1.8,-1.8)
        elif state is 'half_close':
            AmbiClawFingerOrder.__init__(self,-1.0,-1.0,-1.8,-1.8)
        else:
            AmbiClawFingerOrder.__init__(self,-1.8,-1.8,-1.8,-1.8)
            raise smach.InvalidUserCodeError("FingersState : Could not execute : unknown state %s",state)
        
# Use this state to set claws pre created states such as : open, close ...   
# for any of those states the fingers will be maintained closed (so take care if they are already opened)
class ClawsOnlyState(AmbiClawFingerOrder):
    def __init__(self,state):
        if state is 'open':
            AmbiClawFingerOrder.__init__(self,-1.8,-1.8,0.5,0.5)
        elif state is 'close':            
            AmbiClawFingerOrder.__init__(self,-1.8,-1.8,-1.8,-1.8)
        elif state is 'totem_left':
            AmbiClawFingerOrder.__init__(self,-1.8,-1.8,0.0,-1.8)
        elif state is 'totem_right':
            AmbiClawFingerOrder.__init__(self,-1.8,-1.8,-1.8,0.0)
        elif state is 'open_left':
            AmbiClawFingerOrder.__init__(self,-1.8,-1.8,0.5,-1.8)
        elif state is 'open_right':
            AmbiClawFingerOrder.__init__(self,-1.8,-1.8,-1.8,0.5)
        elif state is 'half_close':
            AmbiClawFingerOrder.__init__(self,-1.8,-1.8,-1.0,-1.0)
        else:
            AmbiClawFingerOrder.__init__(self,-1.8,-1.8,-1.8,-1.8)
            raise smach.InvalidUserCodeError("FingersState : Could not execute : unknown state %s",state)
        

# Use this state to set claws and finger pre created states such as : open, close ...   
class FingerClawState(AmbiClawFingerOrder):
    def __init__(self,state):
        if state is 'open':
            AmbiClawFingerOrder.__init__(self,0.5,0.5,0.5,0.5)
        elif state is 'close':            
            AmbiClawFingerOrder.__init__(self,-1.8,-1.8,-1.8,-1.8)
        elif state is 'half_close':
            AmbiClawFingerOrder.__init__(self,-1.0,-1.0,-1.0,-1.0)
        elif state is 'half_open':
            AmbiClawFingerOrder.__init__(self,0.0,0.0,0.0,0.0)
        else:
            AmbiClawFingerOrder.__init__(self,-1.8,-1.8,-1.8,-1.8)
            raise smach.InvalidUserCodeError("FingersState : Could not execute : unknown state %s",state)
        
