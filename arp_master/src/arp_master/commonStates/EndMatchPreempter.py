#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *

# This state allows you to check if the end match timer is fired. It should be present in *every* operationnal state machine
class EndMatchPreempter(PreemptiveCyclicState):
    #on passe outre papa pour ne pas enregistrer la transition timeout
    def __init__(self,offset):
        smach.StateMachine.__init__(self,outcomes=['endMatch'])
        self.preemptiveStates=[]
        self.initClients()
        self.timeout=0
        self.match_duration=rospy.get_param("/match_duration")+offset

    def preemptionCondition(self):
        if (rospy.get_rostime()-Data.start_time).to_sec()>self.match_duration:
            return True
        else:
            return False
       
    def executeTransitions(self):
        return 'endMatch'