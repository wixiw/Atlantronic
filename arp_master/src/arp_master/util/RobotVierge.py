#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
import rospy
from math import *
from UtilARD import *

#il faut voir ca comme un namespace
class RobotVierge:

    @staticmethod
    def getParams():
        try:
            pass #rien
        except KeyError:
            rospy.logerr("RobotVierge: Failed to find rosparams.") 
            
            
class AmbiControlPointRed:
    def __init__(self,x,y,theta,color):
        if color=='red':
            self.x=x
            self.y=y
            self.theta=theta
        elif color=='purple':
            self.x=x
            self.y=-y
            self.theta=normalizeAngle(-theta)
        else:
            self.x=0
            self.y=0
            self.theta=0
            rospy.loginfo("AmbiPoseRed : default case : color (%s)  not defined !!",color)