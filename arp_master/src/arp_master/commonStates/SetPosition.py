#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
import math

from arp_master.fsmFramework import *

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
        self.result = False;
        
        if self.xi=="FREE":
            xi=0
        else:
            xi=self.xi
        if self.yi=="FREE":
            yi=0
        else:
            yi=self.yi
        if self.thetai=="FREE":
            thetai=0
        else:
            thetai=self.thetai  
            
        if math.fabs(xi) > 2 or math.fabs(yi) > 2 or math.fabs(thetai) > 4 :
            rospy.logerr("SetPosition is done with incorrect range values, check units %f %f %f", xi, yi, thetai)
            self.result = False;
        else:
            poseDepart=AmbiPoseYellow(xi,yi,thetai,Data.color)
            
            if self.xi=="FREE":
                poseDepart.x=Inputs.getx()
            if self.yi=="FREE":
                poseDepart.y=Inputs.gety()
            if self.thetai=="FREE":
                poseDepart.theta=Inputs.gettheta()            
            
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