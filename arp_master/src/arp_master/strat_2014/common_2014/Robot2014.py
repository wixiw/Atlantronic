#!/usr/bin/env python
# -*- coding: latin-1 -*-

import roslib; roslib.load_manifest('arp_master')
from arp_master.util.RobotVierge import * 
from arp_master.util.TableVierge import * 

#Robot2014 is never instanciated, it is more a namespace (or a bad designed singleton) than a class.
#Robot2014 should receive every year specific stuff (such as actuators/sensors states and strategic memories)
class Robot2014(RobotVierge):
    
    #dernier delai en s avant la fin du match pour entrer dans l'etat EOG
    #TODO WLA : a remplacer par une machine independante MatchData
    SWITCH_TO_EOG_DELAY = 1
    
    nbBallInLeftCanon = 3
    nbBallInRightCanon = 3

    
    motionSpeeds = {
                    'Carefull' : 0.3,
                    'Average'  : 0.5,
                    'Fast'     : 1.0
                    }
    
    
    shootDelta = 0.040
    
    #
    # Dynamixels configuration
    #
    dynamixelList = [                       
                             'LeftFinger',
                             'RightFinger',
                             'LeftCannonStocker',
                             'RightCannonStocker'
                             ]
        
    dynamixelMaxTorqueList = {                       
                         'LeftFinger':        75,
                         'RightFinger':       75,
                         'LeftCannonStocker':       45,
                         'RightCannonStocker':      45
                         }
    
    #Configuration of reference suction powers
    suctionPower = { 'HOLD':70,
                     'IDLE':0 }
    
    #Configuration of Finger reference positions
    fingerLeftYellowPos = {         'UP'    : -0.2,
                                    'SEARCH': 1.2,
                                    'DOWN'  : 1.45,
                                    'FLOOR' : 1.45}
    
    #Configuration of Cannon reference positions
    cannonFingerSpeed = {           'STOPPED' : 0.0,
                                    'SHOOT'   : 2.25}
       
    cannonStockerLeftYellowPos = {  'LOADING'     : 1.0,
                                    'UNLOADING'   : 0.0,
                                    'SHOWREADY'   : 0.7
                                    }
