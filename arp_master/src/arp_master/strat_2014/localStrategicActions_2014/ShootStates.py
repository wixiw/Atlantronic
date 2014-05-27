#!/usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *

from arp_master.util import *
from arp_master.fsmFramework import *
from arp_master.commonStates import *
from arp_master.strat_2014.common_2014 import *

#===============================================================================
# Here is listed the different shooting positions with Right or Left Cannon
# Do not forget to put a full unload order in ActionSelector2014!!
#===============================================================================

#In order to use this action you have to go to the entry point AmbiShootMammoth.getEntryPoint()

class AmbiShootMammoth(LocalStrategicAction):
    
    @staticmethod
    def getEntryYellowPoseStatic(p_side, p_opponent_side = False):
        
        #Choix du cannon
        if p_side is 'Left':
            x = 0.160
            h = -3*pi/4
            y = 0.480
        elif p_side is 'Right':
            x = 0.100
            y = 0.542
            h = -3*pi/4
        else:
            return None
        
        #Si on visder le mammouth oppose on symmetrise les positions
        if p_opponent_side is True:
            x = -x
            h = pi - h
        
        return Pose2D(x,y,h)
        
        
    def getEntryYellowPose(self):
        return self.getEntryYellowPoseStatic(self.cannon_side, self.opponent_side)
    
    # @param String p_cannon_side                    : defines on the Yellow configuration which cannon to use (Left or Right)
    # @param Boolean p_opponent_side (optionnal)     : if this is set to True, the shoot is done on the opposite side mammouth  
    def __init__(self, p_cannon_side, p_opponent_side = False):
        LocalStrategicAction.__init__(self, Robot2014.SWITCH_TO_EOG_DELAY)
        
        #remember cannon side
        self.cannon_side = p_cannon_side
        #remember opponent side
        self.opponent_side = p_opponent_side
        
        with self:      

            LocalStrategicAction.add('Shoot',
                      AmbiShootOneBall(p_cannon_side),
                      transitions={'shot':'TurnShoot_2', 'blocked':'failed'})
            self.setInitialState('Shoot')
            
# 2x additional shots on Mammoths, turn of 0.085 RAD = 5 DEG

             # Turn Shoot_2
            LocalStrategicAction.add('TurnShoot_2',
                      AmbiTurnOrder(-pi/4 + 0.08),
                      transitions={'succeeded':'Shoot_2', 'timeout':'Shoot_2'})
            
            # Shoot_2            
            LocalStrategicAction.add('Shoot_2',
                      AmbiShootOneBall(p_cannon_side),
                      transitions={'shot':'TurnShoot_3', 'blocked':'failed'})
            
             # Turn Shoot_3
            LocalStrategicAction.add('TurnShoot_3',
                      AmbiTurnOrder(-pi/4 - 0.08),
                      transitions={'succeeded':'Shoot_3', 'timeout':'Shoot_3'})
            
            # Shoot_3           
            LocalStrategicAction.add('Shoot_3',
                      AmbiShootOneBall(p_cannon_side),
                      transitions={'shot':'succeeded', 'blocked':'failed'})

            
            