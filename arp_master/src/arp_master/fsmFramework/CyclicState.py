#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy

import smach
import smach_ros
import smach_msgs


from arp_master.util.Inputs import Inputs
from arp_master.util.Data import Data

#import module for ros services
from arp_core.srv import SetPosition
from arp_core.srv import SetColor
from arp_hml.srv import SetMotorPower
from arp_hml.srv import SetMotorMode
from arp_ods.srv import SetVMax

# the cyclicstate is a modification of the smach standard state machine, so as it is SYNCHRONOUS
# the cyclic state also implements all interfaces with outside of arp_master
# the cyclic state may have an internal timer that fires a timeout transition to prevent being stalled (timeout in seconds)

class CyclicState(smach.StateMachine):
    def __init__(self,outcomes):
        outcomes.append('timeout')
        smach.StateMachine.__init__(self,outcomes)
        self.preemptiveStates=[]
        self.initClients()
        self.timeout=0
    
    # the strategic derivation of smach functions. allow to be synchrone and to call functions that will be overrided by my children
    def execute(self,userdata):
        self.timeIn=rospy.get_rostime().secs
        Inputs.update()
        self.executeIn()
        while(not rospy.is_shutdown()):
            self.executeWhile()
            
            #check if preemption transitions
            for p in self.preemptiveStates:
                label=p[0]
                state=p[1]
                preempted=state.preemptionCondition()
                if preempted:
                    return label


            #check if the timeout is fired
            if (self.timeout != 0 and rospy.get_rostime().secs-self.timeIn>self.timeout):
                return 'timeout'
            #normal transitions
            trans=self.executeTransitions()
            if trans!=None:
                self.executeOut()
                return trans
            Data.stateMachineRate.sleep()
            Inputs.update()
        rospy.logerr("boucle d'etat cassee par le shutdown")
        return "shutdown"
        
    # if executeIn, While or Out are not overiden by children, then they will just do nothing.
    def executeIn(self):
        return
    
    def executeWhile(self):
        return
    
    def executeOut(self):
        return
    
    # executeTransitions is not defined, as it shall be ALWAYS overriden

    # initialisation of the ros clients
    # creation of handy functions for my children
    def initClients(self):
        self.initSetPositionClient()
        self.initSetMotorPower()
        self.initSetDrivingMotorPower()
        self.initSetSteeringMotorPower()
        self.initSetDrivingMotorOperationMode()
        self.initSetSteeringMotorOperationMode()
        self.initSetVMaxClient()
        self.initConfigureColorClient()

    def initSetVMaxClient(self):
        self.setVMax_srv=rospy.ServiceProxy("MotionControl/setVMax",SetVMax)
        
    def initConfigureColorClient(self):
        self.configureColor_srv=rospy.ServiceProxy("/Localizator/setColor",SetColor)
        
    def initSetPositionClient(self):
        self.setPosition_loc=rospy.ServiceProxy("Localizator/setPosition",SetPosition)
        self.setPosition_simu=rospy.ServiceProxy("/Ubiquity/setRealSimulPosition",SetPosition)
        
    def setPosition(self,x,y,theta):
        self.setPosition_loc(x,y,theta)
        #try:
            #self.setPosition_simu(x,y,theta)
       # except rospy.ServiceException, e:
        #    rospy.logerr("Position could not be set on simulator")
    
    
    #----------------------------------------------------
    #recuperation et mapping du service pour mettre la puissance sur les moteurs
    def initSetMotorPower(self):
        self.setMotorPower_srv=rospy.ServiceProxy("Ubiquity/setMotorPower",SetMotorPower)
    def enablePower(self):
        try:
            self.setMotorPower_srv(True)
            return True;
        except rospy.ServiceException, e:
            rospy.logerr("Position could not enable MotorPower")
            return False
    def disablePower(self):
        try:
            self.setMotorPower_srv(False)
            return True;
        except rospy.ServiceException, e:
            rospy.logerr("Position could not disable MotorPower")
            return False
        
    def initSetDrivingMotorPower(self):
        self.setDrivingMotorPower_srv=rospy.ServiceProxy("Ubiquity/setDrivingMotorPower",SetMotorPower)
    def enableDrivingPower(self):
        try:
            self.setDrivingMotorPower_srv(True)
            return True;
        except rospy.ServiceException, e:
            rospy.logerr("Position could not enable Driving MotorPower")
            return False
    def DrivingPowerdisableDrive(self):
        try:
            self.setDrivingMotorPower_srv(False)
            return True;
        except rospy.ServiceException, e:
            rospy.logerr("Position could not disable Driving MotorPower")
            return False
        
    def initSetSteeringMotorPower(self):
        self.setSteeringMotorPower_srv=rospy.ServiceProxy("Ubiquity/setSteeringMotorPower",SetMotorPower)
    def enableSteeringPower(self):
        try:
            self.setSteeringMotorPower_srv(True)
            return True;
        except rospy.ServiceException, e:
            rospy.logerr("Position could not enable Steering MotorPower")
            return False
    def disableSteeringPower(self):
        self.setSteeringMotorPower_srv(False)
        try:
            self.setSteeringMotorPower_srv(False)
            return True;
        except rospy.ServiceException, e:
            rospy.logerr("Position could not disable Steering MotorPower")
            return False
    #----------------------------------------------------
    
    
    def initSetDrivingMotorOperationMode(self):
        self.setDrivingMotorOperationMode_srv=rospy.ServiceProxy("Ubiquity/setDrivingOperationMode",SetMotorMode)
    def setDrivingMotorMode(self,mode):
        try:
            self.setDrivingMotorOperationMode_srv(mode)
            return True;
        except rospy.ServiceException, e:
            rospy.logerr("Position could not switch a Driving motor to a new mode")
            return False
        
    def initSetSteeringMotorOperationMode(self):
        self.setSteeringMotorOperationMode_srv=rospy.ServiceProxy("Ubiquity/setSteeringOperationMode",SetMotorMode)
    def setSteeringMotorMode(self,mode):
        try:
            self.setSteeringMotorOperationMode_srv(mode)
            return True;
        except rospy.ServiceException, e:
            rospy.logerr("Position could not switch a Steering motor to a new mode")
            return False
    
    
    def setVMax(self,v):
        self.setVMax_srv(v,False)
        
    def setVMaxDefault(self):
        self.setVMax_srv(-1.0,True)
        
    def configureColor(self, color):
        try:
            self.configureColor_srv(color)
            return True
        except rospy.ServiceException, e:
            rospy.logerr("configureColor could not switch color")
            return False
        
