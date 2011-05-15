from math import cos
from math import sin
from math import pi
from UtilARD import *

class Robot:
    xPion=142 
    yPion=-87

class Case:
    def __init__(self,i_hor,j_vert):
        self.i=i_hor
        self.j=j_vert
        self.xCenter=self.i*125
        self.yCenter=self.j*125
        
    def coord_WhenPionMilieu(self,theta):
        x=xCenter-(Robot.xPion*cos(theta)-Robot.yPion*sin(theta))
        y=xCenter-(Robot.xPion*sin(theta)+Robot.yPion*cos(theta))
        return (x,y) 
    

class AmbiCaseRed:
    def __init__(self,i_hor,j_vert,color):
        if color=='red':
            super.__init__(self,i_hor,j_vert)
        if color=='blue':
            super.__init__(-self,i_hor,j_vert)
            
class AmbiPoseRed:
    def __init__(self,x,y,theta,color):
        if color=='red':
            self.x=x
            self.y=y
            self.theta=theta
        if color=='blue':
            self.x=-x
            self.y=y
            self.theta=normalizeAngle(theta+pi)
        
        