#!/usr/bin/env python

#libraries for ROS
import roslib; roslib.load_manifest('arp_master')

from arp_master import *
import rospy

class PrintStratInfo(smach.State):
    def __init__(self):
        smach.State.__init__(self,['ok'])
    def execute(self,userdata):
        Table2012.printStratInfo()
        return 'ok'     
      
class PrintOpponents(smach.State):
    def __init__(self):
        smach.State.__init__(self,['ok'])
    def execute(self,userdata):
        Inputs.getOpponents().printOpponents()
        return 'ok'     

