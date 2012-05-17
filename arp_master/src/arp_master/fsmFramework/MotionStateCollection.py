#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs
import actionlib

from MotionState import *
from CyclicActionState import *
from arp_master.util import *
from arp_ods.msg import *
from math import *

# Use this Order State to quickly add move state in your FSM
# give the target (x,y,theta) in (m,m,rad) of robot's CDG and match color
# Ex : AmbiOmniDirectOrder(1.200, -0.700,pi/2)             
class AmbiOmniDirectOrder(MotionState):
    def __init__(self,x,y,theta, vmax=-1.0):
        MotionState.__init__(self)
        self.x = x
        self.y = y
        self.theta = theta
        self.vmax = vmax
        
    def createAction(self):
        self.pose = AmbiPoseRed(self.x, self.y, self.theta, Data.color)
        self.omnidirect(self.pose.x, self.pose.y, self.pose.theta, self.vmax)

# Use this Order State to quickly add move state in your FSM with a special ControlPoint (CP)
# give the target (x,y,h) in (m,m,rad) of robot's CP=(CPx,CPy,CPh) and match color
# Ex : AmbiOmniDirectOrder(CPx=0,CPy=0,CPh=0, 1.200, -0.700,pi/2)   
class AmbiOmniDirectOrder_cpoint(MotionState):
    def __init__(self,CPx,CPy,CPh,x,y,h):
        MotionState.__init__(self)
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
class OmniDirectOrder(MotionState):
    def __init__(self,x,y,theta):
        MotionState.__init__(self)
        self.x = x
        self.y = y
        self.theta = theta
        
    def createAction(self):
        self.omnidirect(self.x,self.y,self.theta)        


# Use this to call an open loop order 
# OpenLoppOrder(0,0,0, duration=0) 
# Only use this if you are forced to specify a color dependent move (else prefer AmbiOpenLoop)   
class AmbiOpenLoopOrder(MotionState):
    def __init__(self,vx,vy,vh,duration):
        MotionState.__init__(self)
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
class OpenLoopOrder(MotionState):
    def __init__(self,vx,vy,vh,duration):
        MotionState.__init__(self)
        self.vx = vx
        self.vy = vy
        self.vh = vh
        self.duration = duration
        
    def createAction(self):
        self.openloop(self.vx,self.vy,self.vh,self.duration)        
        
        
class ForwardOrder(MotionState):
    def __init__(self,dist):
        MotionState.__init__(self)
        self.dist = dist
    def createAction(self):
        self.forward(self.dist)           
        
class BackwardOrder(MotionState):
    def __init__(self,dist):
        MotionState.__init__(self)
        self.dist = dist
    def createAction(self):
        self.backward(self.dist)            
        
class LeftwardOrder(MotionState):
    def __init__(self,dist):
        MotionState.__init__(self)
        self.dist = dist
    def createAction(self):
        self.leftward(self.dist)                  
        
class RightwardOrder(MotionState):
    def __init__(self,dist):
        MotionState.__init__(self)
        self.dist = dist
    def createAction(self):
        self.rightward(self.dist)   
        
#Use this to turn on your current position.        
class AmbiTurnOrder(MotionState):
    def __init__(self,h):
        MotionState.__init__(self)
        self.h=h
    def createAction(self):
        self.cap( AmbiCapRed(self.h,Data.color).angle )      
   
#You should only use this if you have color dependent stuuf to do           
class TurnOrder(MotionState):
    def __init__(self,h):
        MotionState.__init__(self)
        self.h=h
    def createAction(self):
        self.cap(self.h)          