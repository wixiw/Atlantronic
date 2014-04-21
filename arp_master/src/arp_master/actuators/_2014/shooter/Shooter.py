#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master.strat_2014.common_2014.Robot2014 import *

from arp_master.util import *
from arp_master.fsmFramework import *
from arp_master.commonStates import *
from arp_master.strat_2014 import *
from arp_master.actuators import *

#TODO
##
##
## Remettre le preemptive

class ShooterMainStateMachine(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['end','problem'])
        with self:      
            PreemptiveStateMachine.add('Init',
                      ShooterInit(),
                      transitions={'succeeded':'end', 'timeout':'problem'})



class ShooterInit(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['succeeded','timeout'])
    
    def executeIn(self):
        rospy.loginfo("Shooter Init execute")
    
    def executeTransitions(self):
        rospy.loginfo("ShooterInit transit")
        return 'succeeded'   
                    