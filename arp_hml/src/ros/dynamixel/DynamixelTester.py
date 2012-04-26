#! /usr/bin/env python

import roslib; roslib.load_manifest('arp_hml')
import rospy
import actionlib

from arp_hml.msg import *

if __name__ == '__main__':
    rospy.init_node('DynamixelTester')
    args = rospy.myargv()
    if (len(args) != 5):
        rospy.logfatal("you should provide 4 arguments to control dynamixels (currently leng(arg=%d))",len(args))
        for arg in args:
            print arg
        goal = ClawOrderGoal(0,0,0,0)
    else:
        rospy.loginfo("you provided 4 arguments : %s %s %s %s", args[1],args[2],args[3],args[4])
        goal = ClawOrderGoal(float(args[1]),float(args[2]),float(args[3]),float(args[4]))
    
    client = actionlib.SimpleActionClient('move_claw', ClawOrderAction)
    client.wait_for_server(rospy.Duration.from_sec(5.0))
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
    
    print(client.get_result()) 
    