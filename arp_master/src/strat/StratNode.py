#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
import rospy
from arp_core.msg import Obstacle
from arp_core.msg import StartColor
from arp_core.msg import Start
    
###########################  TEMPORAL BEHAVIOR

def StratNode():
    global inputList
    
    rospy.init_node('StratNode')
    init()
    rate =rospy.Rate(1)
    
    while not rospy.is_shutdown():
        for input in inputList:
            input.update()
        mainloop()
        rate.sleep()
        

############################# INITIALISATION
def init():
    
    #creation and linking of inputs
    global inputList,obstacle,color,start    
    inputList=[]
    obstacle= Input("obstacle", Obstacle)
    color=Input("color", StartColor)
    start=Input("start", Start)
    #x = Input()
    #y = Input()
    #theta = Input()

    #welcome message
    rospy.loginfo("******************************************************")
    rospy.loginfo("Welcome to Advanced Robotics Platform. I am StratNode.")
    rospy.loginfo("Choose color with button")
    rospy.loginfo("Then plug start")
    rospy.loginfo("Wait for initialisation sequence")
    rospy.loginfo("And unplug start")
    rospy.loginfo("******************************************************")

    
    
############################# MAIN LOOP
def mainloop():
    global obstacle
    rospy.loginfo("obstacle: %i"%obstacle.data.detected)

########################### INPUT CLASS
# this is used to have a buffer on inputs, so that they are not changing during the mainloop
class Input():
    def __init__(self,topicName,msgType):
        global inputList
        self.data=msgType()
        self.data_lastvalue=msgType()
        inputList.append(self)
        rospy.Subscriber(topicName,msgType,self.updateCallBack)
    def update(self):
        self.data=self.data_lastvalue
    def updateCallBack(self, data):
        self.data_lastvalue=data
    
   
########################## EXECUTABLE 
#shall be always at the end ! so that every function is defined before
# main function, called by ros
if __name__ == '__main__':
    try:
        StratNode()
    except rospy.ROSInterruptException: pass