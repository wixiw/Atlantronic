#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
import math

# 
# These states are usefull to set the position of the robot
#
##################################################


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
            poseDepart=AmbiPoseRed(self.xi,self.yi,self.thetai,Data.color)
            self.setPosition(poseDepart.x,poseDepart.y,poseDepart.theta)
            self.result = True;
    
    def executeTransitions(self):
        if self.result == True:
            return 'succeeded'   
        else:
            return 'timeout'     