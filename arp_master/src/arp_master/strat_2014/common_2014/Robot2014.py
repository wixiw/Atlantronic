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
