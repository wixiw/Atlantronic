#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')
from arp_master import *

class Debloque(CyclicActionState):
    def __init__(self,replayDuration):
        CyclicActionState.__init__(self)
        self.replayDuration=replayDuration
        
    def createAction(self):
        self.replay(self.replayDuration)  