#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
import rospy

from math import cos
from math import sin
from math import pi
from UtilARD import *

class Robot:
    xPion=0.142 
    yPion=-0.087

class Case:
    #attention a cette numerotation tres intelligente !
    #la premiere case c'est 1, la suivante 3, la suivante 5
    #ca permet d'avoir une symetrie sur les numeros de cases ET le repere
    def __init__(self,i_hor,j_vert):
        self.i=i_hor
        self.j=j_vert
        self.xCenter=self.i*(0.350/2)
        self.yCenter=self.j*(0.350/2)
        
    def coord_WhenPionMilieu(self,theta):
        x=self.xCenter-(Robot.xPion*cos(theta)-Robot.yPion*sin(theta))
        y=self.yCenter-(Robot.xPion*sin(theta)+Robot.yPion*cos(theta))
        return (x,y) 
    

class AmbiCaseRed(Case):
    def __init__(self,i_hor,j_vert,color):
        if color=='red':
            Case.__init__(self,i_hor,j_vert)
        if color=='blue':
            Case.__init__(self,-i_hor,j_vert)
            
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
        
        