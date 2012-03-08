#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs

from Inputs import Inputs
from Data import Data

#import module for ros services
from arp_core.srv import SetPosition
from arp_hml.srv import SetMotorPower
from arp_ods.srv import SetVMax

# the cyclicstate is a modification of the smach standard state machine, so as it is SYNCHRONOUS
# the cyclic state also implements all interfaces with outside of arp_master

class CyclicState(smach.StateMachine):
    def __init__(self,outcomes):
        smach.StateMachine.__init__(self,outcomes)
        self.preemptiveStates=[]
        self.initClients()
    
    # the strategic derivation of smach functions. allow to be synchrone and to call functions that will be overrided by my children
    def execute(self,userdata):
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
             
            #normal transitions
            trans=self.executeTransitions()
            if trans!=None:
                self.executeOut()
                return trans
            Data.stateMachineRate.sleep()
            Inputs.update()
        rospy.logerr("boucle d'etat cassee par le shutdown")
        
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
        self.initSetVMaxClient()

    def initSetVMaxClient(self):
        self.setVMax_srv=rospy.ServiceProxy("MotionControl/setVMax",SetVMax)
        
    def initSetPositionClient(self):
        self.setPosition_loc=rospy.ServiceProxy("Localizator/setPosition",SetPosition)
        self.setPosition_simu=rospy.ServiceProxy("PhysicsSimu/setPosition",SetPosition)
        
    def setPosition(self,x,y,theta):
        self.setPosition_loc(x,y,theta)
        try:
            self.setPosition_simu(x,y,theta)
        except rospy.ServiceException, e:
            rospy.logerr("Position could not be set on simulator")
    
    def initSetMotorPower(self):
        self.setMotorPower_srv=rospy.ServiceProxy("Protokrot/setMotorPower",SetMotorPower)
    
    def enableDrive(self):
        self.setMotorPower_srv(True)
        
    def disableDrive(self):
        self.setMotorPower_srv(False)
        
    def setVMax(self,v):
        self.setVMax_srv(v,False)
        
    def setVMaxDefault(self):
        self.setVMax_srv(0,True)
        

        
        
