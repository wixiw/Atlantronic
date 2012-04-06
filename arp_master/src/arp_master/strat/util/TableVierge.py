#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
import rospy
from rospy import loginfo

from math import *
from UtilARD import *

#represent the robot dimensions    
class Robot:
    @staticmethod
    def init():
        Robot.DIST_BACK=rospy.get_param('/Ubiquity/DIST_BACK')

class Table:
    @staticmethod
    def init():
        Table.HWALL_Y=rospy.get_param('/Table/HWALL_Y')
        Table.VWALL_X=rospy.get_param('/Table/VWALL_X')
        
    @staticmethod
    def isOnTable(x,y):
        if x<Table.VWALL_X-0.350 and x>-Table.VWALL_X+0.350 and y<Table.HWALL_Y-0.200 and y>-Table.HWALL_Y+0.300 :
            return True
        else:
            return False
        
class AmbiPoseRed:
    def __init__(self,x,y,theta,color):
        if color=='red':
            self.x=x
            self.y=y
            self.theta=theta
        if color=='blue':
            self.x=-x
            self.y=y
            self.theta=normalizeAngle(pi-theta)
            
class AmbiCapRed:
    def __init__(self,angle,color):
        if color=='red':
            self.angle=angle
        if color=='blue':
            self.angle=normalizeAngle(pi-angle)  