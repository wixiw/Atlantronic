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
    
    fingerTorque = 40
    cannonFingerTorque = 35
    
    
    #
    # Dynamixels configuration
    #
    dynamixelList = [                       
                             'LeftCannonFinger',
                             'LeftCannonStocker',
                             'RightCannonFinger',
                             'RightCannonStocker',
                             'LeftFinger',
                             'RightFinger'
                             ]
        
    dynamixelMaxTorqueList = {                       
                         'LeftCannonFinger':        0,#reglage a faire
                         'RightCannonFinger':       0,#reglage a faire
                         'LeftCannonStocker':       35,
                         'RightCannonStocker':      35,
                         'LeftFinger':              40,
                         'RightFinger':             40
                         }
    
    dynamixelMinPosList = {                       
                         'LeftCannonFinger':        0,#reglage a faire
                         'RightCannonFinger':       0,#reglage a faire
                         'LeftCannonStocker':       -1.0,
                         'RightCannonStocker':      -1.0,
                         'LeftFinger':              -0.2,
                         'RightFinger':             0.2
                         }
    
    dynamixelMaxPosList = {                       
                         'LeftCannonFinger':        0,#reglage a faire
                         'RightCannonFinger':       0,#reglage a faire
                         'LeftCannonStocker':       0, 
                         'RightCannonStocker':      0,
                         'LeftFinger':              1.4,
                         'RightFinger':             -1.4
                         }
    
    #Configuration of Finger reference positions
    fingerLeftYellowPos = { 'UP'    : dynamixelMinPosList['LeftFinger'],
                            'DOWN'  : dynamixelMaxPosList['LeftFinger']}
    
    #Configuration of reference suction powers
    suctionPower = { 'HOLD':100,
                     'IDLE':0 }
    
    #Configuration of Cannon reference positions
    cannonFingerLeftYellowPos = {   'ARMED'       : 0.0,
                                    'GOINFRONT'   : 0.0,
                                    'SHOOT'       : 0.0}
    cannonStockerLeftYellowPos = {  'LOADING'     : dynamixelMinPosList['LeftCannonStocker'],
                                    'UNLOADING'   : dynamixelMaxPosList['LeftCannonStocker']}
