#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
import rospy
from rospy import loginfo

from math import cos
from math import sin
from math import pi
from math import floor
from math import atan2
from UtilARD import *

class Direction:
    def __init__(self,di,dj):
        self.di=di
        self.dj=dj
        
    def getInverse(self):
        return self.getRotated(pi)
    
    def getRotated(self,angle):
        return Direction(int(round(cos(self.angle()+angle))),int(round(sin(self.angle()+angle))))
    
    def angle(self):
        return atan2(self.dj,self.di)    
    
    #me dis si je suis dans une direction horizontale ou verticale
    def isOrtho(self):
        if abs(self.di)+abs(self.dj)==1:
            return True
        else:
            return False
        
    def __repr__(self):
        return "Direction(%i,%i) "%(self.di,self.dj)

        
# give him an angle, it will tell you what "direction" you have on the chess board
def getDirection4Q(anglerobot):
    if -pi/4<anglerobot<=pi/4:
        return Direction(1,0)
    if pi/4<anglerobot<=3*pi/4:
        return Direction(0,1)
    if 3*pi/4<anglerobot or anglerobot<=-3*pi/4 :
        return Direction(-1,0)
    if -3*pi/4<anglerobot<=-pi/4:
        return Direction(0,-1)

    
def getDirection8Q(anglerobot):
    if -pi/8<anglerobot<=pi/8:
        return Direction(1,0)
    if pi/8<anglerobot<=3*pi/8:
        return Direction(1,1)
    if 3*pi/8<anglerobot<=5*pi/8:
        return Direction(0,1)
    if 5*pi/8<anglerobot<=7*pi/8:
        return Direction(-1,1)
    if 7*pi/8<anglerobot or anglerobot<=-7*pi/8 :
        return Direction(-1,0)
    if -7*pi/8<anglerobot<=-5*pi/8:
        return Direction(-1,-1)
    if -5*pi/8<anglerobot<=-3*pi/8:
        return Direction(0,-1)
    if -3*pi/8<anglerobot<=-pi/8:
        return Direction(1,-1)
  

#give him a position, it will return the case on which you are
def getCase(x,y):
    return Case(int(1+2*floor(x/0.350)),int(1+2*floor(y/0.350)))    
    
#represent the robot dimensions    
class Robot:
    xPion=0.142 
    yPion=-0.087
    @staticmethod
    def init():
        Robot.DIST_BACK=rospy.get_param('/Protokrot/DIST_BACK')

class Table:
    @staticmethod
    def init():
        Table.HWALL_Y=rospy.get_param('/Table/HWALL_Y')
        Table.VWALL_X=rospy.get_param('/Table/VWALL_X')

class Case:
    #attention a cette numerotation tres intelligente !
    #la premiere case c'est 1, la suivante 3, la suivante 5
    #ca permet d'avoir une symetrie sur les numeros de cases ET le repere
    
    #c'est un immutable, tout comme direction
    # ca ne fait que renvoyer une nouvelle instance de case si on lui demande, par exemple, la plus proche 
    def __init__(self,i_hor,j_vert):
        self.i=int(i_hor)
        self.j=int(j_vert)
        self.xCenter=self.i*(0.350/2)
        self.yCenter=self.j*(0.350/2)
        
    def coord_WhenPionMilieu(self,theta):
        x=self.xCenter-(Robot.xPion*cos(theta)-Robot.yPion*sin(theta))
        y=self.yCenter-(Robot.xPion*sin(theta)+Robot.yPion*cos(theta))
        return (x,y) 
    
    def color(self):
        if (self.i/2+self.j/2)%2==1:
            return 'blue'
        else:
            return 'red'
    
    def getClosestInDirection(self,dir,color):
        # dir could be (1,1)(-1,0)... etc, meaning "-1 in X and 0 in y"
        i_test=self.i
        j_test=self.j
        inc_i=dir.di*2
        inc_j=dir.dj*2
        while(True):
            i_test+=inc_i
            j_test+=inc_j
            case=Case(i_test,j_test)
            if case.inTable()==False:
                break
            if color=='any':
                return case
            if case.color()==color:
                return case
            
         
    def getFurthestInDirection(self,dir,color):
        i_test=self.i
        j_test=self.j
        inc_i=dir.di*2
        inc_j=dir.dj*2
        last_good_case=None
        while(True):
            i_test+=inc_i
            j_test+=inc_j
            case=Case(i_test,j_test)
            if case.inTable()==False:
                break
            if color=='any':
                last_good_case= case
            if case.color()==color:
                last_good_case= case
                
        return last_good_case   
                
    #will tell if the considered case in within the table            
    def inTable(self):
        # caution: the last row is not considered "in the table" for silly walk
        if self.i>5 or self.j>5 or self.i<-5 or self.j<-3:
            return False
        else:
            return True
        
    #return angle for silly walk, i.e. direction pointing center of table, from this case
    def dirForSillyWalk(self):
        if self.i>0 and self.j>0:
            return Direction(-1,-1)
        if self.i<0 and self.j>0:
            return Direction(1,-1)
        if self.i<0 and self.j<0:
            return Direction(1,1)
        if self.i>0 and self.j<0:
            return Direction(-1,1)
        
    def __repr__(self):
        return "Case(%i,%i) (%s)"%(self.i,self.j,self.color())
    
    def __eq__(self,other):
        if other==None:
            return False
        if self.i==other.i and self.j==other.j:
            return True
        else:
            return False


class AmbiCaseRed(Case):
    def __init__(self,i_hor,j_vert,color):
        if color=='red':
            Case.__init__(self,i_hor,j_vert)
        if color=='blue':
            Case.__init__(self,-i_hor,j_vert)
    
class AmbiCapRed:
    def __init__(self,angle,color):
        if color=='red':
            self.angle=angle
        if color=='blue':
            self.angle=normalizeAngle(pi-angle)   
            
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
        
        