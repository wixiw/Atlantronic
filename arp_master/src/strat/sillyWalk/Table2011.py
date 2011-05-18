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

#give him a position, it will return the case on which you are
def getCase(x,y):
    return Case(1+2*floor(x/350),1+2*floor(y/350))

#represent the robot dimensions    
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
    
    def color(self):
        if (self.i/2+self.j/2)%2==1:
            return 'red'
        else:
            return 'blue'
    
    def getClosestInDirection(dir,color):
        # dir could be (1,1)(-1,0)... etc, meaning "-1 in X and 0 in y"
        i_test=self.i
        j_test=self.j
        inc_i=dir(0)*2
        inc_j=dir(1)*2
        while(True):
            i_test+=inc_i
            j_test+=inc_j
            case=Case(i_test,j_test)
            if case.inTable==False:
                break
            if color=='any':
                return case
            if case.color()==color:
                return case
         
    def getFurthestInDirection(dir,color):
        i_test=self.i
        j_test=self.j
        inc_i=dir(0)*2
        inc_j=dir(1)*2
        last_good_case=None
        while(True):
            i_test+=inc_i
            j_test+=inc_j
            case=Case(i_test,j_test)
            if case.inTable==False:
                break
            if color=='any':
                last_good_case= case
            if case.color()==color:
                last_good_case= case
                
        return last_good_case   
                

    #will tell if the considered case in within the table            
    def inTable(self):
        if self.i>5 or self.j>5 or self.i<-5 or self.j<-5:
            return False
        else:
            return True

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
        
        