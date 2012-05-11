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
    def createMotionControlAction(self,x,y,theta,move_type,passe):
        self.createMotionControlAction_cpoint(0,0,0,x,y,theta,move_type,passe,0,0,0,0)
 
     # motioncontrol action creator with a control point
    def createMotionControlAction_cpoint(self,x_cpoint,y_cpoint,theta_cpoint,x,y,theta,move_type,passe,x_speed,y_speed,theta_speed,openloop_duration):
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
        
        # THIS IS BLOCKING ! <<<<<<<<<<<<<<<<<<<<<<< !!!!!!!!!!!!!!
        self.client.wait_for_server()
        self.client.cancel_all_goals
        self.client.send_goal(goal)
        
        self.actionCreated=True       
        
        
    # these are useful motions functions that allow not to give all parameters  
    def forward(self,dist):
        self.createMotionControlAction(Inputs.getx()+dist*cos(Inputs.gettheta()),
                                       Inputs.gety()+dist*sin(Inputs.gettheta()),
                                       Inputs.gettheta(),
                                       'OMNIDIRECT',False)
        
    def backward(self,dist):
        self.createMotionControlAction(Inputs.getx()+dist*cos(Inputs.gettheta()+pi),
                                       Inputs.gety()+dist*sin(Inputs.gettheta()+pi),
                                       Inputs.gettheta(),
                                       'OMNIDIRECT',False)
    def left(self,dist):
        self.createMotionControlAction(Inputs.getx()+dist*cos(Inputs.gettheta()+pi/2),
                                       Inputs.gety()+dist*sin(Inputs.gettheta()+pi/2),
                                       Inputs.gettheta(),
                                       'OMNIDIRECT',False)
        
    def right(self,dist):
        self.createMotionControlAction(Inputs.getx()+dist*cos(Inputs.gettheta()-pi/2),
                                       Inputs.gety()+dist*sin(Inputs.gettheta()-pi/2),
                                       Inputs.gettheta(),
                                       'OMNIDIRECT',False)

    def omnidirect(self,x,y,theta):
        self.createMotionControlAction_cpoint(-0.0583,0,0,
                                              x,y,theta,
                                              'OMNIDIRECT',False,
                                              0,0,0,0)
    
    def omnidirect_cpoint(self,x_cpoint,y_cpoint,theta_cpoint,x,y,theta):
        self.createMotionControlAction_cpoint(x_cpoint,y_cpoint,theta_cpoint,
                                              x,y,theta,
                                              'OMNIDIRECT',False,
                                              0,0,0,0)    
        
    def cap(self,theta):
        self.createMotionControlAction_cpoint(-0.0583,0,0,
                                              Inputs.getx()-0.0583*cos(theta),Inputs.gety()-0.0583*sin(theta),theta,
                                              'OMNIDIRECT',False,
                                              0,0,0,0)
        
    def openloop(self,x_speed,y_speed,theta_speed,openloop_duration):
        self.createMotionControlAction_cpoint(-0.0583,0,0,
                                              0,0,0,
                                              'OPENLOOP',False,
                                              x_speed,y_speed,theta_speed,openloop_duration)
        
    def openloop_cpoint(self,x_cpoint,y_cpoint,theta_cpoint,
                        x_speed,y_speed,theta_speed,
                        openloop_duration):
        self.createMotionControlAction_cpoint(x_cpoint,y_cpoint,theta_cpoint,
                                              0,0,0,
                                              'OPENLOOP',False,
                                              x_speed,y_speed,theta_speed,openloop_duration)
        
    def replay(self,replay_duration):
        self.createMotionControlAction_cpoint(0,0,0,
                                              0,0,0,
                                              'REPLAY',False,
                                              0,0,0,replay_duration)
    
    