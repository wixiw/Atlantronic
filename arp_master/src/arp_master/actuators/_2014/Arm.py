#!/usr/bin/env python
# -*- coding: latin-1 -*-

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master.strat_2014.common_2014.Robot2014 import *

from arp_master.util import *
from arp_master.fsmFramework import *
from arp_master.commonStates import *
from arp_master.strat_2014 import *
from arp_master.actuators import *

            
#
# Use this state to wait for an object to be present in the suction cup
#
class ArmWaitForObjectPresent(ReceiveFromTopic):
    def __init__(self):
        ReceiveFromTopic.__init__(self, "/Ubiquity/ArmPump/object_present", Bool,
                                  outcomes=['object_present']) 
        
    #The topic is only listened when entering the state to prevent useless trigger when not active       
    def executeIn(self):
        ReceiveFromTopic.executeIn(self)
        return
        
    def executeTransitions(self):
        #if no message has been received yet, just wait again
        if self.msg is None:
            return
        #end with success if last message is a True value
        if self.msg.data is True:
            return 'object_present'  
        #else continue to wait


#
# Use this state to drive the pump on the arm
# @param Integerp_power :     the suction power of the pump in [50;100] in percent
#
class ArmPumpCommand(StaticSendOnTopic):
    def __init__(self, p_power):
        StaticSendOnTopic.__init__(self, "/Ubiquity/ArmPump/suction_power", UInt8, UInt8(p_power))   
        
