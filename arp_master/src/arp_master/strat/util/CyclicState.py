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
from arp_ods.srv import SetVMax
from arp_rlu.srv import FindRoyalFamily

class CyclicState(smach.StateMachine):
    def __init__(self,outcomes):
        smach.StateMachine.__init__(self,outcomes)
        self.preemptiveStates=[]
        self.initClients()
    
    def initClients(self):
        self.initSetPositionClient()
        self.initSetMotorPower()
        self.initEstimatePositionClient()
        self.initSetVMaxClient()
        self.initFindRoyalFamilyClient()
    
    def waitForStart(self):
        rospy.logerr("wait for start")
        while (Data.lastStart==Inputs.getstart()):
            Data.stateMachineRate.sleep()
            Inputs.update()
        rospy.logerr("end of wait for start")
        Data.lastStart=Inputs.getstart()   
        
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
    
    def initSetVMaxClient(self):
        self.setVMax_srv=rospy.ServiceProxy("MotionControl/setVMax",SetVMax)
        
    def initSetPositionClient(self):
        self.setPosition_loc=rospy.ServiceProxy("Localizator/setPosition",SetPosition)
        self.setPosition_simu=rospy.ServiceProxy("PhysicsSimu/setPosition",SetPosition)
        
    def initEstimatePositionClient(self):
        self.estimatePosition_srv=rospy.ServiceProxy("/ReLocalizator/EstimatePosition",EstimatePosition)

    def initFindRoyalFamilyClient(self):
          self.findRoyalFamily_srv=rospy.ServiceProxy("/ChessboardReader/FindRoyalFamily",FindRoyalFamily)

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
                deltaX=answer.estimatedX-Inputs.getx()
                deltaY=answer.estimatedY-Inputs.gety()
                deltaTheta=answer.estimatedTheta-Inputs.gettheta()
                
                rospy.loginfo("RELOC>> Pose:  x=%.3f  y=%.3f,  theta=%.3f"%(Inputs.getx(),Inputs.gety(),Inputs.gettheta()))
                rospy.loginfo("RELOC>> Estimate:  x=%.3f  y=%.3f,  theta=%.3f"%(answer.estimatedX,answer.estimatedY,answer.estimatedTheta))
                rospy.loginfo("RELOC>> delta Pose: delta x=%.3f delta y=%.3f, delta theta=%.3f"%(deltaX,deltaY,deltaTheta))
                
                if (abs(deltaX)<0.300 and abs(deltaY)<0.300 and abs(deltaTheta)<pi/4):
                    self.setPosition(answer.estimatedX,answer.estimatedY,answer.estimatedTheta)
                    rospy.loginfo("RELOC>> RELOCALISATION SUCCESS")
                else: 
                    rospy.logwarn("RELOC>> TEEEELLLLEEETRANSPORTATIION !!! Reloc refusee. Boris, regarde ce qu'il se passe...")

        except rospy.ServiceException, e:
            rospy.logerr("RELOC>> Exception on relocation")
    
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
        
    def findRoyalFamily(self):
        try:
            rospy.loginfo("ROYALFAMILY>> calling find royal family")
            result=self.findRoyalFamily_srv(Inputs.getx(),Inputs.gety(),Inputs.gettheta(),Data.color)
            if result.confidence1>0  or result.confidence2>0:
                if result.confidence1>0 and 0<=result.figure1 <=3: 
                    rospy.loginfo("ROYALFAMILY 1>> ok")
                    Data.listStatusPionsBord[result.figure1]='FIGURE'
                if result.confidence2>0 and 0<=result.figure2 <= 3:
                    rospy.loginfo("ROYALFAMILY 2>> ok")        
                    Data.listStatusPionsBord[result.figure2]='FIGURE'
                #case de merde
                if Data.listStatusPionsBord[0]=='FIGURE'  and Data.color=='red':
                    Data.listStatusPionsBord[0]='NORMAL'
            else:
                rospy.loginfo("ROYALFAMILY>> not found")
                rospy.logerr("ROYALFAMILY>> I officially declare 2 as king ")
                Data.listStatusPionsBord[2]='FIGURE'
            return result
        except rospy.ServiceException, e:
            rospy.logerr("ROYALFAMILY>> Exception when calling service or computing answer")
            rospy.logerr("ROYALFAMILY>> I officially declare  2 as king")
            Data.listStatusPionsBord[2]='FIGURE'
        
        
