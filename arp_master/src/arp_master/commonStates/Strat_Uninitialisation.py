#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
import os
from arp_master.fsmFramework import *
from arp_core.msg import Beep

#
# This is the default a5 level state for any strategy
#
######################################################

class Uninitialisation(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self,outcomes=['endUninitialisation'])
        with self:
            smach.StateMachine.add('UninitialisationState',
                      UninitialisationState(),
                      transitions={'ok':'endUninitialisation','timeout':'endUninitialisation'})
      
class UninitialisationState(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['ok'])
        self.pub = rospy.Publisher('/Master/beep', Beep)

    def executeIn(self):
        self.result = self.disablePower()
        self.pub.publish(Beep(300,3,1))
        os.system("sh /opt/ard/arp_core/script/linux/match_finished.sh")
        
    def executeTransitions(self):
        return 'ok'     
    
    
