#! /usr/bin/env python

import roslib; roslib.load_manifest('arp_hml')
import rospy
import actionlib

# import the definition of the messages
from arp_core.msg import OpponentsList

class SpeedReducer:
    def __init__(self):
        rospy.init_node('SpeedReducer')
        self.opponentsMsg= Inputs.createInput("Localizator/opponents_detected", OpponentsList)
    
        try:
            self.cropping_distance = rospy.get_param('/SpeedReducer/cropping_distance', Float64)
            self.reduced_min_speed = rospy.get_param('/SpeedReducer/reduced_min_speed', Float64)
        except KeyError:
            rospy.logerr("Failed to find cropping_distance rosparams.") 
    
    #a appeler des que l'objet est construit pour commencer a travailler. Appel bloquant
    def start(self):
        rospy.loginfo("DynamixelManager : started")
        while not rospy.is_shutdown():
            opp = Opponents(Inputs.opponentsInput.data)
            distance = opp.getClosestDistance();
            if distance <= cropping_distance:
                rospy.loginfo("Opponent reducespeed")
               
                
            rospy.sleep(0.1)
    
    
if __name__ == '__main__':
    try:
        sppedreducer = SpeedReducer()
        sppedreducer.start()
    except rospy.ROSInterruptException: pass