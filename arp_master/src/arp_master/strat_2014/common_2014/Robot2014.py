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
                         'LeftCannonFinger':        70,
                         'RightCannonFinger':       70,
                         'LeftCannonStocker':       30,
                         'RightCannonStocker':      30,
                         'LeftFinger':              45,
                         'RightFinger':             45
                         }
    
    dynamixelMinPosList = {                       
                         'LeftCannonFinger':        -3,
                         'RightCannonFinger':       1,
                         'LeftCannonStocker':       1.0,
                         'RightCannonStocker':      -1.0,
                         'LeftFinger':              -0.2,
                         'RightFinger':             0.2
                         }
    
    dynamixelMaxPosList = {                       
                         'LeftCannonFinger':        0,
                         'RightCannonFinger':       -1,
                         'LeftCannonStocker':       0.0,
                         'RightCannonStocker':      0.0,
                         'LeftFinger':              1.4,
                         'RightFinger':             -1.4
                         }
    
    #Configuration of Finger reference positions
    fingerLeftYellowPos = { 'UP'    : dynamixelMinPosList['LeftFinger'],
                            'DOWN'  : dynamixelMaxPosList['LeftFinger'],
                            'FLOOR' : dynamixelMaxPosList['LeftFinger']}
    
    #Configuration of reference suction powers
    suctionPower = { 'HOLD':100,
                     'IDLE':0 }
    
    #Configuration of Cannon reference positions
    cannonFingerLeftYellowPos = {   'ARMED'       : 0.9,
                                    'GOINFRONT'   : -0.8,
                                    'SHOOT'       : 1.5}
    cannonStockerLeftYellowPos = {  'LOADING'     : dynamixelMinPosList['LeftCannonStocker'],
                                    'UNLOADING'   : dynamixelMaxPosList['LeftCannonStocker']}
