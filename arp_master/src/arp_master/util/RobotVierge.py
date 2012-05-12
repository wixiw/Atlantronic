#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
import rospy

#il faut voir ca comme un namespace
class RobotVierge:

    @staticmethod
    def getParams():
        try:
            pass #rien
        except KeyError:
            rospy.logerr("RobotVierge: Failed to find rosparams.") 