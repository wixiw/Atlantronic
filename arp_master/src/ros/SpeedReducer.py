#! /usr/bin/env python

import roslib; roslib.load_manifest('arp_master')
import rospy
import actionlib

# import the definition of the messages
from arp_core.msg import OpponentsList
from arp_core.msg import Pose
from std_msgs.msg import Float64
from arp_ods.srv import SetVMax
from arp_master import *

#This speed reducer is the very simple avoidance system that reduce velocity when someone is close to the robot
class SpeedReducer:
    def __init__(self):
        rospy.init_node('SpeedReducer')
        self.opponentsTopic = rospy.Subscriber("Localizator/opponents_detected", OpponentsList,self.updateOpponentCallBack)
        self.poseTopic      = rospy.Subscriber("Localizator/pose", Pose,self.updatePoseCallBack)
        #NOTE : les calculs sont fait au centre des tourelles, ca meriterait d'etre fait au centre du robot.
        self.opponentsMgs = OpponentsList()
        self.pose = Pose()
        
        #la vitesse est limitee a reduced_min_speed si l'adversaire est a moins de min_distance de nous
        #sinon la vitesse est limitee en fonction de (distanceOpponent-reduced_min_distance) a condition 
        # d'etre dans le cone +/-search_angle/2
        #   pour faire une rampe de freinage de valeur decc
        try:
            self.min_distance = rospy.get_param('/SpeedReducer/min_distance', Float64)
            self.reduced_min_speed = rospy.get_param('/SpeedReducer/reduced_min_speed', Float64)
            self.search_angle = rospy.get_param('/SpeedReducer/search_angle', Float64)
            self.decc = rospy.get_param('/SpeedReducer/decc', Float64)
        except KeyError:
            rospy.logerr("Failed to find cropping_distance rosparams.") 
            raise RuntimeError

        rospy.wait_for_service('MotionControl/setVMax')            
        self.setVMax_srv=rospy.ServiceProxy("MotionControl/setVMax",SetVMax)
    
    #a appeler des que l'objet est construit pour commencer a travailler. Appel bloquant
    def start(self):
        rospy.loginfo("SpeedReducer : started")
        while not rospy.is_shutdown():
            opp = Opponents(self.opponentsMgs,Point(self.pose.x, self.pose.y ))
            #opp.printOpponents()
            distance = opp.closest_distance;
            
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
                    
            rospy.sleep(0.050) 
    
    def updateOpponentCallBack(self, oppList):
        self.opponentsMgs = oppList
    def updatePoseCallBack(self, pose):
        self.pose = pose
    
    def setVmax(self,v):
        try:
            self.setVMax_srv(v,False)
        except:
            rospy.logerr("Failed to setVmax");
            
    def unsetVmax(self):
        try:
            self.setVMax_srv(0.0,True)
        except:
            rospy.logerr("Failed to unsetVmax");
    
if __name__ == '__main__':
    try:
        speedreducer = SpeedReducer()
        speedreducer.start()
    except rospy.ROSInterruptException: pass