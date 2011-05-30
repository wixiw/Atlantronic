#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
import rospy

# import the definition of the messages
from arp_core.msg import Obstacle
from arp_core.msg import StartColor
from arp_core.msg import Start
from arp_core.msg import Pose


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
    
    @staticmethod
    def link():
        Inputs.obstacleInput= Inputs.createInput("ObstacleDetector/front_obstacle", Obstacle)
        Inputs.colorInput=Inputs.createInput("Protokrot/color", StartColor)
        Inputs.startInput=Inputs.createInput("Protokrot/start", Start)
        Inputs.poseInput=Inputs.createInput("Localizator/pose", Pose)
        Inputs.rearObstacleInput=Inputs.createInput("ObstacleDetector/rear_obstacle", Obstacle)
    
    @staticmethod
    def createInput(name,type):
        inputcreated=Input(name,type)
        Inputs.inputList.append(inputcreated)
        return inputcreated
    
    @staticmethod    
    def update():
        for input in Inputs.inputList:
                input.update()
    @staticmethod
    def getcolor():
        return Inputs.colorInput.data.color
    
    @staticmethod
    def getstart():
        return Inputs.startInput.data.go
    
    @staticmethod
    def getobstacle():
        return Inputs.obstacleInput.data.detected

    @staticmethod
    def getRearObstacle():
        return Inputs.rearObstacleInput.data.detected
    
    @staticmethod
    def getx():
        return Inputs.poseInput.data.x
    
    @staticmethod
    def gety():
        return Inputs.poseInput.data.y
    
    @staticmethod
    def gettheta():
        return Inputs.poseInput.data.theta
    
    
  
