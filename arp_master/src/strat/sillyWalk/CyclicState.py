#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs

from Inputs import Inputs
from Data import Data

from arp_core.srv import Spawn

class CyclicState(smach.StateMachine):
    def __init__(self,outcomes):
        smach.StateMachine.__init__(self,outcomes)
        self.initSetPositionClient()
        
    def execute(self,userdata):
        Inputs.update()
        self.executeIn()
        while(not rospy.is_shutdown()):
            self.executeWhile()
            trans=self.executeTransitions()
            if trans!=None:
                self.executeOut()
                return trans
            Data.stateMachineRate.sleep()
            Inputs.update()
        rospy.logerr("boucle d'etat cassee par le shutdown")
        
    #si In, Out et While ne sont pas declarees par l'etat derive alors elles ne feront rien
    #par contre je ne decris pas transitions qui elle doit toujours etre declaree
    
    def executeIn(self):
        return
    
    def executeWhile(self):
        return
    
    def executeOut(self):
        return
    
    def initSetPositionClient(self):
        self.setPosition_loc=rospy.ServiceProxy("Localizator/respawn",Spawn)
        #self.setPosition_loclaser=rospy.ServiceProxy("CSM/respawn",Spawn)
        self.setPosition_simu=rospy.ServiceProxy("PhysicsSimu/respawn",Spawn)
        
    def setPosition(self,x,y,theta):
        self.setPosition_loc(x,y,theta)
        #self.setPosition_loclaser(x,y,theta)
        self.setPosition_simu(x,y,theta)
        
        
