#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs
import actionlib


from CyclicState import CyclicState
from arp_master.util import *
from arp_ods.msg import *
from math import *

# ** You should not have to use this state **
# Prefer the use of utilities at the bottom of the file like AmbiOmniDirectOrder, Replay
class CyclicActionState(CyclicState):
    
    def __init__(self):
        CyclicState.__init__(self,outcomes=['succeeded','timeout'])
        self.timeout = 10
        self.lastStart=None
        try:
            self.blinding_period=rospy.get_param("/blinding_period")
        except KeyError:
            rospy.logerr("Failed to find a rosparam : /blinding_period") 
            self.blinding_period=0
   
    
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
        
        if state==actionlib.GoalStatus.ABORTED or state==actionlib.GoalStatus.REJECTED or state==actionlib.GoalStatus.LOST or state==actionlib.GoalStatus.PREEMPTED or self.isFrontObstacle() or self.isRearObstacle():
            self.client.cancel_all_goals()
            return 'timeout'  
        #all others are considered "waiting"
        #Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST
    
    #check for obstacles
    def isFrontObstacle(self):
        if Inputs.getObstacle()==1 and rospy.get_rostime().secs-Data.timeObstacleInAction>self.blinding_period and Inputs.getLinearVelocity()>0.010:
            Data.timeObstacleInAction=rospy.get_rostime().secs
            return True
        else:
            return False
    def isRearObstacle(self):    
        if Inputs.getRearObstacle()==1 and rospy.get_rostime().secs-Data.timeRearObstacleInAction>self.blinding_period and Inputs.getLinearVelocity()<-0.010:
            Data.timeRearObstacleInAction=rospy.get_rostime().secs
            return True
        else:
            return False     
             
    # generic motioncontrol action creator.
    def createMotionControlAction(self,x,y,theta,move_type,passe,max_speed):
        self.createMotionControlAction_cpoint(0,0,0,x,y,theta,move_type,passe,0,0,0,0,max_speed)
 
     # motioncontrol action creator with a control point
    def createMotionControlAction_cpoint(self,x_cpoint,y_cpoint,theta_cpoint,x,y,theta,move_type,passe,x_speed,y_speed,theta_speed,openloop_duration,max_speed):
        self.client = actionlib.SimpleActionClient('MotionControl', OrderAction)
        goal=OrderGoal()
        goal.x_des=x
        goal.y_des=y
        goal.theta_des=theta
        goal.x_cpoint=x_cpoint
        goal.y_cpoint=y_cpoint
        goal.theta_cpoint=theta_cpoint
        goal.move_type=move_type
        goal.passe=passe
        goal.x_speed=x_speed
        goal.y_speed=y_speed
        goal.theta_speed=theta_speed
        goal.openloop_duration=openloop_duration
        goal.max_speed=max_speed
        
        self.client.wait_for_server(rospy.Duration.from_sec(5.0))
        self.client.cancel_all_goals
        self.client.send_goal(goal)
        
        self.actionCreated=True       
        
        
    # these are useful motions functions that allow not to give all parameters  
    def forward(self,dist,v_max=-1.0):
        self.createMotionControlAction(Inputs.getx()+dist*cos(Inputs.gettheta()),
                                       Inputs.gety()+dist*sin(Inputs.gettheta()),
                                       Inputs.gettheta(),
                                       'OMNIDIRECT',False,v_max)
        
    def backward(self,dist,v_max=-1.0):
        self.createMotionControlAction(Inputs.getx()+dist*cos(Inputs.gettheta()+pi),
                                       Inputs.gety()+dist*sin(Inputs.gettheta()+pi),
                                       Inputs.gettheta(),
                                       'OMNIDIRECT',False,v_max)
    def leftward(self,dist,v_max=-1.0):
        self.createMotionControlAction(Inputs.getx()+dist*cos(Inputs.gettheta()+pi/2),
                                       Inputs.gety()+dist*sin(Inputs.gettheta()+pi/2),
                                       Inputs.gettheta(),
                                       'OMNIDIRECT',False,v_max)
        
    def rightward(self,dist,v_max=-1.0):
        self.createMotionControlAction(Inputs.getx()+dist*cos(Inputs.gettheta()-pi/2),
                                       Inputs.gety()+dist*sin(Inputs.gettheta()-pi/2),
                                       Inputs.gettheta(),
                                       'OMNIDIRECT',False,v_max)

    def omnidirect(self,x,y,theta,v_max=-1.0):
        self.createMotionControlAction_cpoint(-0.0583,0,0,
                                              x,y,theta,
                                              'OMNIDIRECT',False,
                                              0,0,0,0,v_max)
    
    def omnidirect_cpoint(self,x_cpoint,y_cpoint,theta_cpoint,x,y,theta,v_max=-1.0):
        self.createMotionControlAction_cpoint(x_cpoint,y_cpoint,theta_cpoint,
                                              x,y,theta,
                                              'OMNIDIRECT',False,
                                              0,0,0,0,v_max)    
        
    def cap(self,theta,v_max=-1.0):
        self.createMotionControlAction_cpoint(0.0,0,0,
                                              Inputs.getx(),Inputs.gety(),theta,
                                              'OMNIDIRECT',False,
                                              0,0,0,0,v_max)
        
    def openloop(self,x_speed,y_speed,theta_speed,openloop_duration):
        self.createMotionControlAction_cpoint(-0.0583,0,0,
                                              0,0,0,
                                              'OPENLOOP',False,
                                              x_speed,y_speed,theta_speed,openloop_duration,-1.0)
        
    def openloop_cpoint(self,x_cpoint,y_cpoint,theta_cpoint,
                        x_speed,y_speed,theta_speed,
                        openloop_duration):
        self.createMotionControlAction_cpoint(x_cpoint,y_cpoint,theta_cpoint,
                                              0,0,0,
                                              'OPENLOOP',False,
                                              x_speed,y_speed,theta_speed,openloop_duration,-1.0)
        
    def replay(self,replay_duration):
        self.createMotionControlAction_cpoint(0,0,0,
                                              0,0,0,
                                              'REPLAY',False,
                                              0,0,0,replay_duration,-1.0)
    
    
###############################################################################################################################


# Use this Order State to quickly add move state in your FSM
# give the target (x,y,theta) in (m,m,rad) of robot's CDG and match color
# Ex : AmbiOmniDirectOrder(1.200, -0.700,pi/2)             
class AmbiOmniDirectOrder(CyclicActionState):
    def __init__(self,x,y,theta):
        CyclicActionState.__init__(self)
        self.x = x
        self.y = y
        self.theta = theta
        
    def createAction(self):
        self.pose = AmbiPoseRed(self.x, self.y, self.theta, Data.color)
        self.omnidirect(self.pose.x, self.pose.y, self.pose.theta)

# Use this Order State to quickly add move state in your FSM with a special ControlPoint (CP)
# give the target (x,y,h) in (m,m,rad) of robot's CP=(CPx,CPy,CPh) and match color
# Ex : AmbiOmniDirectOrder(CPx=0,CPy=0,CPh=0, 1.200, -0.700,pi/2)   
class AmbiOmniDirectOrder_cpoint(CyclicActionState):
    def __init__(self,CPx,CPy,CPh,x,y,h):
        CyclicActionState.__init__(self)
        self.x = x
        self.y = y
        self.h = h
        self.CPx = CPx
        self.CPy = CPy
        self.CPh = CPh
        
    def createAction(self):
        self.pose = AmbiPoseRed(self.x, self.y, self.h, Data.color)
        self.control_point = AmbiControlPointRed(self.CPx, self.CPy, self.CPh, Data.color)
        self.omnidirect_cpoint(self.control_point.x, self.control_point.y, self.control_point.theta,
                               self.pose.x, self.pose.y, self.pose.theta)


# Use this state to replay the move before the collision that has just block the robot
# replayDuration is the length in seconds of the replay.
# CAUTION : there is no insurrance that a certain distance will be done from the collision... it depends on cases.
class Replay(CyclicActionState):
    def __init__(self,replayDuration):
        CyclicActionState.__init__(self)
        self.replayDuration=replayDuration
        
    def createAction(self):
        self.replay(self.replayDuration)  
    
# Use this Order State to quickly add move state in your FSM (see AmbiOmniDirectOrder for details)
# Only use this if you are forced to specify a color dependent move (else prefer AmbiOmniDirectOrder)    
class OmniDirectOrder(CyclicActionState):
    def __init__(self,x,y,theta):
        CyclicActionState.__init__(self)
        self.x = x
        self.y = y
        self.theta = theta
        
    def createAction(self):
        self.omnidirect(self.x,self.y,self.theta)        


# Use this to call an open loop order 
# OpenLoppOrder(0,0,0, duration=0) 
# Only use this if you are forced to specify a color dependent move (else prefer AmbiOpenLoop)   
class AmbiOpenLoopOrder(CyclicActionState):
    def __init__(self,vx,vy,vh,duration):
        CyclicActionState.__init__(self)
        self.vx = vx
        self.vy = vy
        self.vh = vh
        self.duration = duration
        
    def createAction(self):
        self.speed = AmbiControlPointRed(self.vx, self.vy, self.vh, Data.color)
        self.openloop(self.speed.x,self.speed.y,self.speed.theta,self.duration) 

# Use this to call an open loop order 
# OpenLoppOrder(0,0,0, duration=0) 
# Only use this if you are forced to specify a color dependent move (else prefer AmbiOpenLoop)   
class OpenLoopOrder(CyclicActionState):
    def __init__(self,vx,vy,vh,duration):
        CyclicActionState.__init__(self)
        self.vx = vx
        self.vy = vy
        self.vh = vh
        self.duration = duration
        
    def createAction(self):
        self.openloop(self.vx,self.vy,self.vh,self.duration)        
        
        
class ForwardOrder(CyclicActionState):
    def __init__(self,dist):
        CyclicActionState.__init__(self)
        self.dist = dist
    def createAction(self):
        self.forward(self,self.dist)           
        
class BackwardOrder(CyclicActionState):
    def __init__(self,dist):
        CyclicActionState.__init__(self)
        self.dist = dist
    def createAction(self):
        self.backward(self,self.dist)            
        
class LeftwardOrder(CyclicActionState):
    def __init__(self,dist):
        CyclicActionState.__init__(self)
        self.dist = dist
    def createAction(self):
        self.leftward(self,self.dist)                  
        
class RightwardOrder(CyclicActionState):
    def __init__(self,dist):
        CyclicActionState.__init__(self)
        self.dist = dist
    def createAction(self):
        self.rightward(self,self.dist)            