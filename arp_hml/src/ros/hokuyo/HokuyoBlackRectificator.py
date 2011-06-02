#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_hml')
import rospy
from sensor_msgs.msg import LaserScan
from arp_hml.srv import *

class HokuyoBlackRectificator():
    
    def __init__(self):
        rospy.init_node('HokuyoBlackRectificator', anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.callback)
        
        rospy.spin()

    def callback(self, data):
        rospy.loginfo(rospy.get_name() + "bite !")
        


if __name__ == '__main__':
    try:
        HokuyoBlackRectificator()
    except rospy.ROSInterruptException: pass
    
