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

    def getEntryYellowPose(self):
        return Pose2D(0.000, 0.400, -pi/2);
    
    def __init__(self):
        LocalStrategicAction.__init__(self, Robot2014.SWITCH_TO_EOG_DELAY)
        with self:          
            # Approach Fresco
            LocalStrategicAction.add('GoToFresco',
                      AmbiOmniDirectOrder2(Pose2D(0.000, 1.000 - 0.214 - 0.050, pi+0.428)),
                      transitions={'succeeded':'StickFrescos', 'timeout':'failed'})
            self.setInitialState('GoToFresco')
            
            #Utiliser un ordre d'approche pour gerer la collision avec le mur (peut venir du timeout precedent)
                        
            # Stick Fresco
            LocalStrategicAction.add('StickFrescos',
                      AmbiOmniDirectOrder2(Pose2D(0.000, 1.000 - 0.214 + 0.200, pi+0.428),vmax=0.3),
                      transitions={#a motion success mean that we didn't bumped the wall so it's a problem
                                   'succeeded':'failed', 
                                   #a motion failure mean that we bumped the wall, so it's a success
                                   'timeout':'QuitFresco'})

            # Quit Fresco
            LocalStrategicAction.add('QuitFresco',
                      AmbiOmniDirectOrder2(self.getEntryYellowPose()),
                      transitions={'succeeded':'succeeded', 'timeout':'failed'})
            
        