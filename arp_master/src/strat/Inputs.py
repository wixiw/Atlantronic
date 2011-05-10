#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
import rospy

# import the definition of the messages
from arp_core.msg import Obstacle
from arp_core.msg import StartColor
from arp_core.msg import Start

########################### INPUT CLASS
# this is used to have a buffer on inputs, so that they are not changing during the mainloop
class Input():
    def __init__(self,topicName,msgType):
        global inputList
        self.data=msgType()
        self.data_lastvalue=msgType()
        self.ncall=0
        self.ncall_lastvalue=0
        rospy.Subscriber(topicName,msgType,self.updateCallBack)
    def update(self):
        self.data=self.data_lastvalue
        self.ncall=self.ncall_lastvalue
        self.ncall_lastvalue=0
    def updateCallBack(self, data):
        self.data_lastvalue=data
        self.ncall_lastvalue+=1  
        
########################### INPUTS CLASS
class Inputs:
    
    inputList=[]
    obstacle=0
    color=0
    start=0
    
    def link(self):
        obstacleInput= self.createInput("obstacle", Obstacle)
        Inputs.obstacle=obstacleInput.data.detected
        colorInput=Input("color", StartColor)
        Inputs.color=colorInput.data.color
        startInput=Input("start", Start)
        Inputs.start=startInput.data.go
    
    def createInput(self,name,type):
        inputcreated=Input(name,type)
        Inputs.inputList.append(inputcreated)
        return inputcreated
        
    def update(self):
        for input in Inputs.inputList:
                input.update()
    
    
    
  