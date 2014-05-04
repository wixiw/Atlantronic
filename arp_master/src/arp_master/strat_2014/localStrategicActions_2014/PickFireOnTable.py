#!/usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *

from arp_master.util import *
from arp_master.fsmFramework import *
from arp_master.commonStates import *
from arp_master.strat_2014.common_2014 import *
    
    
    
    
    
    # Go to Red Fire Bot
#            PreemptiveStateMachine.add('GoToRFB',
#                      AmbiOmniDirectOrder2(Pose2D(-0.400 - Robot2014.FRONT_SIDE.x, -0.600, 0), vmax=1.0),
#                      transitions={'succeeded':'PickRFB', 'timeout':'ReverseOrder'})
            
#as initial state is not the preemptive one, it is necessary to add the information here !