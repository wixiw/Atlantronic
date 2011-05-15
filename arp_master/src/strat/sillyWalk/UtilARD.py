from math import pi

def normalizeAngle(angle):
    angle=angle%(2*pi)
    if angle>pi:
        angle= angle - 2*pi
    if angle <= -pi:
        angle=angle+2*pi 
    return angle