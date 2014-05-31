#!/usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *

from arp_master.util import *
from arp_master.fsmFramework import *
from arp_master.commonStates import *
from arp_master.strat_2014.common_2014 import *

#This action stick frescos, as a total of 2.
#In order to use this action you have to go to the entry point StickFrescosState.getEntryPoint()


class StickFrescosState(LocalStrategicAction):

    @staticmethod
    def getEntryYellowPoseStatic():
        return Pose2D(0.000, 0.475, pi+0.428);
    
    def getEntryYellowPose(self):
        return self.getEntryYellowPoseStatic();
    
    def __init__(self):
        LocalStrategicAction.__init__(self, Robot2014.SWITCH_TO_EOG_DELAY)
        with self:          
            # Approach Fresco
            LocalStrategicAction.add('GoToFresco',
                      AmbiOmniDirectOrder2(Pose2D(0.000, 1.000 - Robot2014.LEFT_SIDE.y - 0.150, -pi+0.428), vmax=Robot2014.motionSpeeds['Average']),
                      transitions={'succeeded':'StickFrescos', 'timeout':'failed'})
            self.setInitialState('GoToFresco')
            
            #Utiliser un ordre d'approche pour gerer la collision avec le mur (peut venir du timeout precedent)
                        
            # Stick Fresco
            LocalStrategicAction.add('StickFrescos',
                      AmbiOmniDirectOrder2(Pose2D(0.000, 1.000 - 0.214 + 0.200, -pi+0.428), vmax=Robot2014.motionSpeeds['Carefull']),
                      transitions={#a motion success mean that we didn't bumped the wall so it's a problem
                                   'succeeded':'failed', 
                                   #a motion failure mean that we bumped the wall, so it's a success
                                   'timeout':'QuitFresco'})

            # Quit Fresco
            LocalStrategicAction.add('QuitFresco',
                      AmbiOmniDirectOrder2(self.getEntryYellowPose()),
                      transitions={'succeeded':'succeeded', 'timeout':'failed'})
            
class ReverseStickFrescosState(LocalStrategicAction):

    @staticmethod
    def getEntryYellowPoseStatic():
        return Pose2D(0.000, 0.475, -0.428);
    
    def getEntryYellowPose(self):
        return self.getEntryYellowPoseStatic();
    
    def __init__(self):
        LocalStrategicAction.__init__(self, Robot2014.SWITCH_TO_EOG_DELAY)
        with self:          
            # Approach Fresco
            LocalStrategicAction.add('GoToFresco',
                      AmbiOmniDirectOrder2(Pose2D(0.000, 1.000 - Robot2014.LEFT_SIDE.y - 0.150, -0.428), vmax=Robot2014.motionSpeeds['Average']),
                      transitions={'succeeded':'ReverseStickFrescos', 'timeout':'failed'})
            self.setInitialState('GoToFresco')
            
            #Utiliser un ordre d'approche pour gerer la collision avec le mur (peut venir du timeout precedent)
                        
            # Stick Fresco
            LocalStrategicAction.add('ReverseStickFrescos',
                      AmbiOmniDirectOrder2(Pose2D(0.000, 1.000 - 0.214 + 0.200, -0.428),vmax=Robot2014.motionSpeeds['Carefull']),
                      transitions={#a motion success mean that we didn't bumped the wall so it's a problem
                                   'succeeded':'failed', 
                                   #a motion failure mean that we bumped the wall, so it's a success
                                   'timeout':'ReverseQuitFresco'})

            # Quit Fresco
            LocalStrategicAction.add('ReverseQuitFresco',
                      AmbiOmniDirectOrder2(self.getEntryYellowPose()),
                      transitions={'succeeded':'succeeded', 'timeout':'failed'})
            
                