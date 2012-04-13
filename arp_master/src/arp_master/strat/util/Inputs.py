#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_master')
import rospy
import tf

# import the definition of the messages
from arp_core.msg import Obstacle
from arp_core.msg import StartColor
from arp_core.msg import Start
from arp_core.msg import Pose
from arp_core.msg import Velocity
from std_msgs.msg import Bool
from TableVierge import *


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
        #transfer buffer
        self.data=self.data_lastvalue
        self.ncall=self.ncall_lastvalue
        #reset the number of callback calls
        self.ncall_lastvalue=0
    def updateCallBack(self, data):
        #buffer for data
        self.data_lastvalue=data
        #buffer to known the number of time the callback was called between update() call
        self.ncall_lastvalue+=1  
        
########################### INPUTS CLASS
class Inputs:
    
    inputList=[]

    #function that creates the inputs list
    @staticmethod
    def link():
        Inputs.obstacleInput= Inputs.createInput("ObstacleDetector/front_obstacle", Obstacle)
        Inputs.colorInput=Inputs.createInput("Ubiquity/color", StartColor)
        Inputs.startInput=Inputs.createInput("Ubiquity/start", Start)
        Inputs.poseInput=Inputs.createInput("Localizator/pose", Pose)
        Inputs.rearObstacleInput=Inputs.createInput("ObstacleDetector/rear_obstacle", Obstacle)
        Inputs.listener = tf.TransformListener()
        Inputs.linearVelocityInput=Inputs.createInput("/Command/velocity",Velocity)
        Inputs.deployed=Inputs.createInput("Master/deployed", Bool)
    
    @staticmethod
    def createInput(name,type):
        inputcreated=Input(name,type)
        Inputs.inputList.append(inputcreated)
        return inputcreated
    
    @staticmethod    
    def update():
        for input in Inputs.inputList:
                input.update()
                
    #### functions for easy access of inputs
    
    @staticmethod
    def getcolor():
        return Inputs.colorInput.data.color
    
    @staticmethod
    def getstart():
        return Inputs.startInput.data.go
    
    @staticmethod
    def getdeployed():
        return Inputs.deployed.data.data
    
    #get obstacle will return that there is an obstacle only if the obstacle is on the table
    @staticmethod
    def getObstacle():
        
        if Inputs.obstacleInput.data.detected:
            now = rospy.Time.now()
            
            try:
                Inputs.listener.waitForTransform("/world", "/front_obstacle", rospy.Time(0), rospy.Duration(0.5))
                (trans,rot) = Inputs.listener.lookupTransform("/world", "/front_obstacle", rospy.Time(0))
            except (tf.Exception, tf.LookupException, tf.ConnectivityException) as ex:
                rospy.loginfo("obstacle mais exception lors de calcul tf machintruc:"+str(ex))
                return 0
            
            if (Table.isOnTable(trans[0],trans[1])):
                rospy.loginfo("obstacle detecte sur la table")
                return 1
            else:
                rospy.loginfo("obstacle mais pas sur table")
                return 0
        else:
            return 0

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
    
    @staticmethod
    def getLinearVelocity():
        return Inputs.linearVelocityInput.data.linear
