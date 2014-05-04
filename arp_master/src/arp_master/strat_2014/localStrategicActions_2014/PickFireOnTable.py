#!/usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *

from arp_master.util import *
from arp_master.fsmFramework import *
from arp_master.commonStates import *
from arp_master.strat_2014.common_2014 import *


class PickFireOnTable(LocalStrategicAction):

    def getEntryYellowPose(self):
        return self.pose2D;
    
    def __init__(self, p_pose2D):
        LocalStrategicAction.__init__(self, Robot2014.SWITCH_TO_EOG_DELAY)
        self.pose2D = p_pose2D
        with self:          
            # Approach Fire
            LocalStrategicAction.add('ApproachFire',
                      ForwardOrder(0.050, vmax=0.3),
                      transitions={'succeeded':'PickFire', 'timeout':'PickFire'})
            self.setInitialState('ApproachFire')
            
            #Utiliser un ordre d'approche pour approcher le feu 
                        
            # Pick Fire
            LocalStrategicAction.add('PickFire',
                      WaiterState(2),
                      transitions={'timeout':'succeeded'})