from math import pi
from math import sqrt

def normalizeAngle(angle):
    angle=angle%(2*pi)
    if angle>pi:
        angle= angle - 2*pi
    if angle <= -pi:
        angle=angle+2*pi 
    return angle

def intersectCircle(x0,y0,r0,x1,y1,r1):
    #see http://paulbourke.net/geometry/2circle/ for justif
    d=sqrt((x0-x1)**2+(y0-y1)**2)
    if d>r0+r1:
        return
    
    a = (r0**2 - r1**2 + d**2 ) / (2* d)
    h = sqrt(r0**2 - a**2) 
    
    x2 = x0 + a * ( x1 - x0 ) / d
    y2 = y0 + a * ( y1 - y0 ) / d
    
    x3_1 = x2 + h* ( y1 - y0 ) / d 
    y3_1 = y2 - h* ( x1 - x0 ) / d 
    
    x3_2 = x2 - h* ( y1 - y0 ) / d 
    y3_2 = y2 + h* ( x1 - x0 ) / d 
    
    return (x3_1,y3_1,x3_2,y3_2)




    