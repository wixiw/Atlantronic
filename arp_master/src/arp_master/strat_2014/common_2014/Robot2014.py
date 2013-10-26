#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
from arp_master.util.RobotVierge import * 
from arp_master.util.TableVierge import * 

#il faut voir ca comme un namespace
class Robot2014(RobotVierge):


    #position du CDG dans le reperere du robot
    CDG_POSE = Point(-0.0583,0)
    
    #position des extremites du robot
    LEFT_SIDE = Point(0,0.212)
    RIGHT_SIDE = Point(0,-0.212)
    REAR_SIDE = Point(-0.233,0)
    #mesure pifometrique a verifier
    FRONT_SIDE = Point(0.065,0)
    
    #temps en s avant la fin pour faire la fin de jeu
    END_GAME_DELAY = 1
    
    
