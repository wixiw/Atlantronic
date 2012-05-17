#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
import rospy
from rospy import loginfo
from math import *
from UtilARD import *

#il faut voir ca comme un namespace
class TableVierge:
    HWALL_Y=1.000
    VWALL_X=1.500
    
    @staticmethod
    def getParams():
        try:
            HWALL_Y=rospy.get_param('/Table/HWALL_Y')
            VWALL_X=rospy.get_param('/Table/VWALL_X')
        except KeyError:
            rospy.logerr("TableVierge: Failed to find init_pos rosparams.") 

    @staticmethod   
    def isOnTable(x,y):
        if x<VWALL_X-0.200 and x>-VWALL_X+0.200 and y<HWALL_Y-0.200 and y>-HWALL_Y+0.200 :
            return True
        else:
            return False
    
    #donne le quart de table dans lequel est le point x,y (en fonction de la couleur)
    @staticmethod
    def getTableHalf(x,y, color):
        if color == 'red':
            rospy.loginfo("Color is red")
            if x < 0 and y < 0:
                return 'farBot'
            if x < 0 and y >= 0:
                return 'farTop'
            if x >= 0 and y >= 0:
                return 'closeTop'     
            #if x >= 0 and y < 0:
            return 'closeBot'    
        elif color == 'purple':
            rospy.loginfo("Color is not red")
            if x < 0 and y < 0:
                return 'closeBot'
            if x < 0 and y >= 0:
                return 'closeTop'
            if x >= 0 and y >= 0:
                return 'farTop'     
            #if x >= 0 and y < 0:
            return 'farBot'    
        else:
            rospy.loginfo("getTableHalf: Color is unknown : %s", color)
            return 'closeTop'  
        
    #renvoit le quart de table oppose        
    @staticmethod
    def getOppositeHalf(table_half):
        if table_half == "closeTop":
            return "farBot"
        elif table_half == "closeBot":
            return "farTop"
        elif table_half == "farBot":
            return "closeTop"
        elif table_half == "farTop":
            return "closeBot"
        else:
            rospy.loginfo("getOppositeHalf: wrong table half %s", table_half)
            return 'closeTop' 
        
class AmbiPoseRed:
    def __init__(self,x,y,theta,color):
        if color=='red':
            self.x=x
            self.y=y
            self.theta=theta
        elif color=='purple':
            self.x=-x
            self.y=y
            self.theta=normalizeAngle(pi-theta)
        else:
            self.x=0
            self.y=0
            self.theta=0
            rospy.logerr("AmbiPoseRed : default case : color not defined !!")
            
class AmbiCapRed:
    def __init__(self,angle,color):
        if color=='red':
            self.angle=angle
        elif color=='purple':
                self.angle=normalizeAngle(pi-angle)  
        else:
            self.angle=0
            rospy.logerr("AmbiCapRed : default case : color not defined !!")
            
            

class Point(object):
    """Point class with public x and y attributes 
        Un point n'as pas de cap a priori. Il sera utilise pour definir des positions de reference sur la table
        Ce sont en general des positions que l'ont abordera depuis plusieurs directions differentes
    """
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y
 
    def dist(self, p):
        """return the Euclidian distance between self and p"""
        dx = self.x - p.x
        dy = self.y - p.y
        return sqrt(dx*dx + dy*dy)
 
    def angle(self, p):
        """return the Euclidian distance between self and p"""
        dx = self.x - p.x
        dy = self.y - p.y
        return atan2(dy,dx)
 
    def reset(self):
        self.x = 0
        self.y = 0

    def __add__(self, p):
        """return a new point found by adding self and p. This method is
        called by e.g. p+q for points p and q"""
        return Point(self.x+p.x, self.y+p.y)
 
    def __repr__(self):
        """return a string representation of this point. This method is
        called by the repr() function, and
        also the str() function. It should produce a string that, when
        evaluated, returns a point with the 
        same data."""
        return 'Point(%d,%d)' % (self.x, self.y)     
    
    def toAmbiPose(self,theta,color):       
        """ permet de convertir le point en une ambicase. """
        return AmbiPoseRed(self.x,self.y,theta,color)