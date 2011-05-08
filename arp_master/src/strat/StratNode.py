#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
import rospy

def StratNode():
    #pub = rospy.Publisher('chatter', String)
    rospy.init_node('StratNode')
    while not rospy.is_shutdown():
        str = "Bonjour, je suis le stratnode %s"%rospy.get_time()
        rospy.loginfo(str)
        rospy.sleep(5.0)

if __name__ == '__main__':
    try:
        StratNode()
    except rospy.ROSInterruptException: pass
