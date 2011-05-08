#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
import rospy
from arp_core.msg import Obstacle

# variables
ncallback=0
obstacle=0
    
    
###########################  TEMPORAL BEHAVIOR

def StratNode():
    rospy.init_node('StratNode')
    init()
    rate =rospy.Rate(1)
    while not rospy.is_shutdown():
        mainloop()
        rate.sleep()
        

############################# INITIALISATION
def init():    
    rospy.Subscriber("obstacle", Obstacle, obstacleCB)
    
    rospy.loginfo("******************************************************")
    rospy.loginfo("Welcome to Advanced Robotics Platform. I am StratNode.")
    rospy.loginfo("Choose color with button")
    rospy.loginfo("Then plug start")
    rospy.loginfo("Wait for initialisation sequence")
    rospy.loginfo("And unplug start")
    rospy.loginfo("******************************************************")

    
    
############################# MAIN LOOP
def mainloop():
    global ncallback,obstacle
    rospy.loginfo("obstacle=%.3f"%obstacle)
    rospy.loginfo("callback appelee %i fois"%ncallback)
    ncallback=0



########################## CALL BACKS    
def obstacleCB(data):
    global ncallback,obstacle
    obstacle=data.detected
    ncallback+=1
    

########################## EXECUTABLE 
#shall be always at the end ! so that every function is defined before
# main function, called by ros
if __name__ == '__main__':
    try:
        StratNode()
    except rospy.ROSInterruptException: pass