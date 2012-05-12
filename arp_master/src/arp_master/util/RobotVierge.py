#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
import rospy


class RobotVierge:
    def __init__(self):
        pass
    
    def getParams(self):
        try:
            pass #rien
        except KeyError:
            rospy.logerr("RobotVierge: Failed to find rosparams.") 