#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
from arp_master.util.RobotVierge import * 

#il faut voir ca comme un namespace
class Robot2012(RobotVierge):
    
    FINGER_CLOSE = -1.8
    FINGER_HALF_CLOSE = -1.0
    FINGER_HALF_OPEN = 0
    FINGER_OPEN = 0.5
    
    CLAW_CLOSE = -1.8
    CLAW_HALF_CLOSE = -1.0
    CLAW_TOTEM = -0.0
    CLAW_OPEN = 0.5

    #...
    
    
    def __init__(self):
        RobotVierge.__init__(self)
        rospy.loginfo("Init Robot 2012 datas ...")