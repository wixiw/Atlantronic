#!/usr/bin/env python
# -*- coding: latin-1 -*-

import roslib; roslib.load_manifest('arp_master')
import rospy
from math import *
from UtilARD import *

#RobotVierge is never instanciated, it is more a namespace (or a bad designed singleton) than a class.
#RobotVierge shall contains every piece of code that is year-independent and robot related. 
# Repere du robot :
# x est vers l'avant, y vers la gauche, theta est compte de x vers y
class RobotVierge:

    #position du CDG dans le repere du robot
    CDG_POSE = Point(-0.0583,0)
    
    #position des extremites du robot
    LEFT_SIDE = Point(0,0.212)
    RIGHT_SIDE = Point(0,-0.212)
    REAR_SIDE = Point(-0.233,0)
    FRONT_SIDE = Point(0.064,0)
    
    

