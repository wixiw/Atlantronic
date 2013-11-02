#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
from arp_master.util.RobotVierge import * 
from arp_master.util.TableVierge import * 

#il faut voir ca comme un namespace
class Robot2014(RobotVierge):

 #cf robot vierge pour position classiques chassis
    
    #temps en s avant la fin pour faire la fin de jeu
    END_GAME_DELAY = 1
    
    
