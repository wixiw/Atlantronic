#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
import rospy
from math import *
from UtilARD import *

#il faut voir ca comme un namespace
class RobotVierge:

#x est vers l'avant, y vers la droite


    #position du CDG dans le reperere du robot
    CDG_POSE = Point(-0.0583,0)
    
    #position des extremites du robot
    LEFT_SIDE = Point(0,0.212)
    RIGHT_SIDE = Point(0,-0.212)
    REAR_SIDE = Point(-0.233,0)
    #mesure pifometrique a verifier
    FRONT_SIDE = Point(0.065,0)
    
    @staticmethod
    def getParams():
        try:
            pass #rien
        except KeyError:
            rospy.logerr("RobotVierge: Failed to find rosparams.") 
            
            
