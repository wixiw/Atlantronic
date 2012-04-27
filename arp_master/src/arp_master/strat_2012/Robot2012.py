#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
import rospy
from rospy import loginfo

from arp_master.util import RobotVierge

class Robot2012(RobotVierge):
    def truc(self):
        pass