#! /usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import actionlib

# import the definition of the messages
from arp_core.msg import OpponentsList
from arp_core.msg import Pose
from arp_core.msg import MotionTarget
from std_msgs.msg import Float32
from arp_ods.srv import SetVMax
from arp_master.util import *

#This more advanced speed reducer than SpeedReducer, is only slowing if the opponent is in our path.
class SpeedReducerAntico:
    def __init__(self):
        rospy.init_node('SpeedReducerAntico')
        self.opponentsTopic = rospy.Subscriber("/Localizator/opponents_detected", OpponentsList,self.updateOpponentCallBack)
        self.poseTopic      = rospy.Subscriber("/Localizator/pose", Pose,self.updatePoseCallBack)
        self.motionTargetTopic      = rospy.Subscriber("/Master/motionTarget", MotionTarget,self.updateMotionTargetCallBack)
        self.speedLimitPublisher = rospy.Publisher("/Master/setVMax", Float32)
        
        #NOTE : les calculs sont fait au centre des tourelles, ca meriterait d'etre fait au centre du robot.
        self.opponentsMgs = OpponentsList()
        self.pose = Pose()
        self.motionTarget=MotionTarget()
        
        #la vitesse est limitee a reduced_min_speed si l'adversaire est a moins de min_distance de nous
        #sinon la vitesse est limitee en fonction de (distanceOpponent-reduced_min_distance) a condition 
        # d'etre dans le cone +/-search_angle/2
#        #   pour faire une rampe de freinage de valeur decc
#        try:
#            self.min_distance = rospy.get_param('/SpeedReducer/min_distance', Float64)
#            self.reduced_min_speed = rospy.get_param('/SpeedReducer/reduced_min_speed', Float64)
#            self.search_angle = rospy.get_param('/SpeedReducer/search_angle', Float64)
#            self.decc = rospy.get_param('/SpeedReducer/decc', Float64)
#        except KeyError:
#            rospy.logerr("Failed to find cropping_distance rosparams.") 
#            raise RuntimeError

        self.min_distance = 0.550
        self.reduced_min_speed = 0.000
        self.decc = 1.0

    #a appeler des que l'objet est construit pour commencer a travailler. Appel bloquant
    def start(self):
        #rospy.loginfo("SpeedReducer : started")
        
        while not rospy.is_shutdown():
            
            #rospy.loginfo("SpeedReducer : isTranslation %s", self.motionTarget.isTranslation)
            
            if self.motionTarget.isTranslation==True: # en bfcap ou Rewind je fais rien, en translation je checke
            
                opp = Opponents(self.opponentsMgs,Point(self.pose.x, self.pose.y ))
                #opp.printOpponents()
                #distance = opp.closest_distance;
                
                isInTrajectory=False
                opponentInTrajectory=False
                distance=1000.0
                
                #rospy.loginfo("SpeedReducer : opp list size %d", len(opp.list))
                
                for i,potential in enumerate(opp.list): # je boucle sur les opponents detectes
                    potentialPosition=Point(potential.x,potential.y)
                    myPosition=Point(self.pose.x,self.pose.y)
                    motionTargetPosition=Point(self.motionTarget.target.x,self.motionTarget.target.y)
                    
                    #rospy.loginfo("["+str(i)+"] SpeedReducer : potentialPosition %s", potentialPosition)
                    #rospy.loginfo("["+str(i)+"] SpeedReducer : myPosition %s", myPosition)
                    #rospy.loginfo("["+str(i)+"] SpeedReducer : motionTargetPosition %s", motionTargetPosition)
                    
                    isInTrajectory=isInRectangle(potentialPosition,myPosition,motionTargetPosition,0.900,0.300)
                    
                    #rospy.loginfo("["+str(i)+"] SpeedReducer : isInTrajectory %s", isInTrajectory)
                    
                    if isInTrajectory: # il est sur ma traj ?
                        opponentInTrajectory=True
                        distancePotential=myPosition.dist(potentialPosition)
                        if distancePotential<distance: # c'est le plus pres ?
                            distance=distancePotential #je note la distance de l'adversaire le plus proche
                
                #rospy.loginfo("SpeedReducer : isInTrajectory %s", isInTrajectory)
                
                # y avait-il qqn sur mon traj ?
                if opponentInTrajectory:
                    #rospy.loginfo("SpeedReducer : adversaire dans ma trajectoire")
                    #rospy.loginfo("SpeedReducer : a une distance de %s",distance)
                    #l'adversaire est tres proche
                    if distance <= self.min_distance:
                        #rospy.loginfo("SpeedReducer : min %s", self.reduced_min_speed)
                        self.setVmax(self.reduced_min_speed)
                    #on ralentit en fonction de la distance de l'adversaire
                    else:
                        dx = distance - self.min_distance
                        dh = normalizeAngle(opp.closest_angle - self.pose.theta)
                        v = sqrt(2.0*self.decc*dx) + self.reduced_min_speed
                        #rospy.loginfo("SpeedReducer : prop %s", v)
                        self.setVmax(v)
                        
                        #if fabs(dh) <= self.search_angle/2.0:
                        #    self.setVmax(v)
                            #rospy.loginfo("SpeedReducer : prop %s", v)
                        #else:
                        #    self.unsetVmax()
                            #rospy.loginfo("SpeedReducer : prop out of angle (dh %s = %s - %d)",dh,opp.closest_angle,self.pose.theta)
                else:
                    #rospy.loginfo("SpeedReducer : unset ")
                    self.unsetVmax()
                
            # fin du test si j'etais en translation
                        
            rospy.sleep(0.020) 
    
    def updateOpponentCallBack(self, oppList):
        #print("Opponent : nb=" + str(oppList.nbOpponents))
        self.opponentsMgs = oppList
    def updatePoseCallBack(self, pose):
        #print("Pose : " + str(pose))
        self.pose = pose
        
    def updateMotionTargetCallBack(self, motionTarget):
        #print("MotionTarget")
        self.motionTarget=motionTarget
    
    def setVmax(self,v):
        #print("Limit Speed " + str(v))
        self.speedLimitPublisher.publish(Float32(v))
            
    def unsetVmax(self):
        #print("UnLimit Speed ")
        self.speedLimitPublisher.publish(Float32(-1.0))
    
if __name__ == '__main__':
    try:
        speedreducerantico = SpeedReducerAntico()
        speedreducerantico.start()
    except rospy.ROSInterruptException: pass