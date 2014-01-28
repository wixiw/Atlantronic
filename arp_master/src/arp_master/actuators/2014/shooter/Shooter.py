#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *

from Table2014 import *
from Robot2014 import *


class Shooter(PreemptiveStateMachine):
    def __init__(self):
        PreemptiveStateMachine.__init__(self,outcomes=['end','problem'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-Robot2014.SWITCH_TO_EOG_DELAY),
                                             transitions={'endMatch':'end'})
            
            PreemptiveStateMachine.add('Init',
                      ShooterInit(),
                      transitions={'succeeded':'end', 'timeout':'problem'})



#Driving power management       
class ShooterInit(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['succeeded','timeout'])
    
    def executeIn(self):
        rospy.loginfo("Shooter Init execute")
    
    def executeTransitions(self):
        rospy.loginfo("ShooterInit transit")
        return 'succeeded'   
                    