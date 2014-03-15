#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
import math

# 
# These states are usefull to set the position of the robot
#
##################################################

class SetPositionState(CyclicState):
    def __init__(self,x,y,theta):
        CyclicState.__init__(self, outcomes=['succeeded'])

        self.xi = x
        self.yi = y
        self.thetai = theta
            
    def executeIn(self):
        if self.xi=="FREE":
            xi=Inputs.getx()
        else:
            xi=self.xi
        if self.yi=="FREE":
            yi=Inputs.gety()
        else:
            yi=self.yi
        if self.thetai=="FREE":
            thetai=Inputs.gettheta()
        else:
            thetai=self.thetai     
            
        self.setPosition(xi,yi,thetai)
        self.result = True;
    
    def executeTransitions(self):
        if self.result == True:
            return 'succeeded'   
        else:
            return 'timeout'    
        

class SetInitialPosition(CyclicState):
    def __init__(self,x,y,theta):
        CyclicState.__init__(self, outcomes=['succeeded'])
        self.xi = x
        self.yi = y
        self.thetai = theta
            
    
    def executeIn(self):
        self.result = False;
        if math.fabs(self.xi) > 2 or math.fabs(self.yi) > 2 or math.fabs(self.thetai) > 4 :
            rospy.logerr("SetInitialPosition is done with incorrect range values, check units %f %f %f", self.xi, self.yi, self.thetai)
            self.result = False;
        else:
            poseDepart=AmbiPoseYellow(self.xi,self.yi,self.thetai,Data.color)
            self.setPosition(poseDepart.x,poseDepart.y,poseDepart.theta)
            self.setGyroPosition(poseDepart.theta);
            self.result = True;
    
    def executeTransitions(self):
        if self.result == True:
            return 'succeeded'   
        else:
            return 'timeout'     
        
        
        
class StartGyroCalibrationState(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['succeeded'])
    
    def executeIn(self):
        self.result = self.startGyroCalibration();
    
    def executeTransitions(self):
        if self.result == True:
            return 'succeeded'   
        else:
            return 'timeout'  
        
        
class StopGyroCalibrationState(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['succeeded'])
    
    def executeIn(self):
        self.result = self.stopGyroCalibration();
        
    def executeTransitions(self):
        if self.result == True:
            return 'succeeded'   
        else:
            return 'timeout' 
        
        
class SetGyroPositionState(CyclicState):
    def __init__(self,theta):
        CyclicState.__init__(self, outcomes=['succeeded'])
        self.theta = theta
    
    def executeIn(self):
        self.result = self.setGyroPosition(self.theta);
    
    def executeTransitions(self):
        if self.result == True:
            return 'succeeded'   
        else:
            return 'timeout' 