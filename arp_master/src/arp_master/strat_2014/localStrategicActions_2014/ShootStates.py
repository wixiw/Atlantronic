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
    def getEntryYellowPoseStatic(p_opponent_side = False):
        x = 0.570
        y = 0.425 #position moulineau a 300
        h = -1.95 + Robot2014.shootDelta 
        
        #Si on viser le mammouth oppose on symetrise les positions
        if p_opponent_side is True:
            x = -x
            h = pi - h
        
        return Pose2D(x,y,h)
        
        
    def getEntryYellowPose(self):
        return self.getEntryYellowPoseStatic(self.opponent_side)
    
    # @param String p_cannon_side                    : defines on the Yellow configuration which cannon to use (Left or Right)
    # @param Boolean p_opponent_side (optionnal)     : if this is set to True, the shoot is done on the opposite side mammouth  
    def __init__(self, p_opponent_side = False):
        LocalStrategicAction.__init__(self, Robot2014.SWITCH_TO_EOG_DELAY)
        
        #remember cannon side
        if p_opponent_side is True:
            self.cannon_side = "Left"
        else:
            self.cannon_side = "Right"
        
        with self:   
            PreemptiveStateMachine.add('ApproachBasket',
                      AmbiOmniDirectOrder2(
                                           Pose2D(self.getEntryYellowPoseStatic(p_opponent_side).x, 
                                                  0.800, 
                                                  self.getEntryYellowPoseStatic(p_opponent_side).theta), 
                                           vmax=Robot2014.motionSpeeds['Carefull']),
                      transitions={'succeeded':'AcquireBasketTarget', 'timeout':'AcquireBasketTarget'})
            self.setInitialState('ApproachBasket')

            PreemptiveStateMachine.add('AcquireBasketTarget',
                      AmbiOmniDirectOrder2(self.getEntryYellowPoseStatic(p_opponent_side), 
                                           vmax=Robot2014.motionSpeeds['Carefull']),
                      transitions={'succeeded':'Shoot', 'timeout':'Shoot'})


            LocalStrategicAction.add('Shoot',
                      AmbiShootFirstBall(self.cannon_side,1),
                      transitions={'shot':'TurnShoot_2', 'blocked':'failed'})
            
             # Turn Shoot_2
            LocalStrategicAction.add('TurnShoot_2',
                      AmbiTurnOrder(self.getEntryYellowPoseStatic(p_opponent_side).theta - Robot2014.shootDelta),
                      transitions={'succeeded':'Shoot_2', 'timeout':'Shoot_2'})
            
            # Shoot_2            
            LocalStrategicAction.add('Shoot_2',
                      AmbiShootNBalls(self.cannon_side,1),
                      transitions={'shot':'TurnShoot_3', 'blocked':'failed'})
            
             # Turn Shoot_3
            LocalStrategicAction.add('TurnShoot_3',
                      AmbiTurnOrder(self.getEntryYellowPoseStatic(p_opponent_side).theta - 2*Robot2014.shootDelta),
                      transitions={'succeeded':'Shoot_3', 'timeout':'Shoot_3'})
            
            # Shoot_3           
            LocalStrategicAction.add('Shoot_3',
                      AmbiShootNBalls(self.cannon_side,1),
                      transitions={'shot':'ReturnToEntryPoint', 'blocked':'failed'})

            PreemptiveStateMachine.add('ReturnToEntryPoint',
                      AmbiOmniDirectOrder2(self.getEntryYellowPoseStatic(p_opponent_side), vmax=Robot2014.motionSpeeds['Average']),
                      transitions={'succeeded':'succeeded', 'timeout':'succeeded'})
            