#! /usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import actionlib

# import the definition of the messages
from arp_master import *
from arp_core.msg import Beep
import os

class BeepMachine:
    def __init__(self):
        rospy.init_node('BeepMachine')
        rospy.Subscriber("/Master/beep", Beep, self.callback,  queue_size=5)
        
    def callback(self, data):
        beepCommand = "beep -f " + str(data.frequency) + " -l " + str(data.duration*1000) + " -r " + str(data.repetition)
        rospy.loginfo("Beep requested :" + beepCommand)
        os.system(beepCommand)
        
    def start(self):
        rospy.spin()
    
if __name__ == '__main__':
    try:
        beepMachine = BeepMachine()
        beepMachine.start()
    except rospy.ROSInterruptException: pass