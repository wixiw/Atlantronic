#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
from arp_master.util.RobotVierge import * 

#il faut voir ca comme un namespace
class Robot2012(RobotVierge):
    
    #...
    
    
    def __init__(self):
        RobotVierge.__init__(self)
        rospy.loginfo("Init Robot 2012 datas ...")