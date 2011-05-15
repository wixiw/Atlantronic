#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
import rospy

from math import cos
from math import sin
from math import pi
from UtilARD import *

def getDirection(anglerobot):
    if -pi/4<anglerobot<=pi/4:
        return (1,0)
    if pi/4<anglerobot<=3*pi/4:
        return (0,1)
    if 3*pi/4<anglerobot or anglerobot<=-3*pi/4 :
        return (-1,0)
    if -3*pi/4<anglerobot<=-pi/4:
        return (0,-1)

    
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
    
    def returnColor(self):
        if (self.i/2+self.j/2)%2==1:
            return 'red'
        else:
            return 'blue'
        

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
        
        