#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
from arp_master.util.RobotVierge import * 
from arp_master.util.TableVierge import * 

#il faut voir ca comme un namespace
class Robot2012(RobotVierge):
    
    #positions predefinies des dynamixels des doigts
    FINGER_CLOSE = -1.7
    FINGER_HALF_CLOSE = -1.2
    FINGER_HALF_OPEN = 0
    FINGER_OPEN = 0.5
    
    #positions predefinies des dynamixel des griffes
    CLAW_CLOSE = -1.7
    CLAW_HALF_CLOSE = -1.2
    CLAW_TOTEM = -0.0
    CLAW_OPEN = 0.5

    #position du CDG dans le reperere du robot
    CDG_POSE = Point(-0.0583,0)
    
    #marge entre l'interieur du "E" de la griffe et le totem
    TOTEM_CLAW_MARGIN = 0.022
    
    #position du doigt droit lorsqu'il est ouvert (dans le repere du robot)
    LEFT_FINGER_OPEN_POSE = Point(162,173)
    RIGHT_FINGER_OPEN_POSE = Point(-162,173)
    
    #position des extremites du robot
    LEFT_SIDE = Point(0,212)
    RIGHT_SIDE = Point(0,-212)
    REAR_SIDE = Point(-232.6,0)
    
    #temps en s avant la fin pour faire la fin de jeu
    END_GAME_DELAY = 1
    
    #...
    
