#!/usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import smach
import smach_ros
import smach_msgs

from Inputs import Inputs
from Data import Data

from arp_core.srv import Spawn
from arp_core.srv import SetPosition
from arp_hml.srv import SetMotorPower
from arp_rlu.srv import EstimatePosition

class CyclicState(smach.StateMachine):
    def __init__(self,outcomes):
        smach.StateMachine.__init__(self,outcomes)
        self.preemptiveStates=[]
        self.initSetPositionClient()
        self.initSetMotorPower()
        self.initEstimatePositionClient()
        
        
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
        
    #si In, Out et While ne sont pas declarees par l'etat derive alors elles ne feront rien
    #par contre je ne decris pas transitions qui elle doit toujours etre declaree
    
    def executeIn(self):
        return
    
    def executeWhile(self):
        return
    
    def executeOut(self):
        return
    
    def initSetPositionClient(self):
        self.setPosition_loc=rospy.ServiceProxy("Localizator/setPosition",SetPosition)
        self.setPosition_simu=rospy.ServiceProxy("PhysicsSimu/setPosition",SetPosition)
        
    def initEstimatePositionClient(self):
        self.estimatePosition_srv=rospy.ServiceProxy("/ReLocalizator/EstimatePosition",EstimatePosition)

    def setPosition(self,x,y,theta):
        self.setPosition_loc(x,y,theta)
        try:
            self.setPosition_simu(x,y,theta)
        except rospy.ServiceException, e:
            rospy.logerr("Position could not be set on simulator")
    
    def relocate(self):
        try:
            answer=self.estimatePosition_srv(Inputs.getx(),Inputs.gety(),Inputs.gettheta())
            if answer.quality==-1:
                rospy.loginfo("RELOC>> Relocalisation failure: quality : -1")
            else:
                rospy.loginfo(" Pose:  x=%.3f  y=%.3f,  theta=%.3f"%(Inputs.getx(),Inputs.gety(),Inputs.gettheta()))
                rospy.loginfo(" Estimate:  x=%.3f  y=%.3f,  theta=%.3f"%(answer.estimatedX,answer.estimatedY,answer.estimatedTheta))
                rospy.loginfo("RELOC>> delta Pose: delta x=%.3f delta y=%.3f, delta theta=%.3f"%(answer.estimatedX-Inputs.getx(),answer.estimatedY-Inputs.gety(),answer.estimatedTheta-Inputs.gettheta()))
                self.setPosition(answer.estimatedX,answer.estimatedY,answer.estimatedTheta)
                rospy.loginfo("RELOC>> RELOCALISATION SUCCESS")

        except rospy.ServiceException, e:
            rospy.logerr("RELOC>> Exception on relocation")
    
    def initSetMotorPower(self):
        self.setMotorPower_srv=rospy.ServiceProxy("Protokrot/setMotorPower",SetMotorPower)
    
    def enableDrive(self):
        self.setMotorPower_srv(True,2.0)
        
    def disableDrive(self):
        self.setMotorPower_srv(False,2.0)
        
